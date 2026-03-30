package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.util.Util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends MotorSubsystem {

    /// TUNING

    public static boolean ACCOUNT_FOR_ROBOT_VEL = true; // TODO make private final

    // approximate amount of seconds the fuel spends in the air
    // used to account for robot velocity
    private static final double AIRTIME_ESTIMATE = 1;

    // multiplier to get from target fuel velocity to target flywheel velocity
    // in theory, this would be ~2 to account for the rotation of the fuel as it leaves the shooter
    public static double VELOCITY_MULT = 2.6; //AGATA wants it lower

    // hardcoded offset between the robot angle and the exit angle of the fuel
    // positive means that the fuel exits the robot to the left/counterclockwise of the expected angle
    private static final double ANGLE_OFFSET = Units.degreesToRadians(-3);

    private static final double MAX_VOLTAGE = 11;

    private static final double kP = 0, kI = 0, kD = 0, kS = 0, kV = 0.097; // 0.097;

    /// MEASUREMENTS

    public static final double GRAVITY = 9.8;

    public static final double FLYWHEEL_CIRC = Units.inchesToMeters(4 * Math.PI);
    public static final double SHOOTER_ANGLE = Units.degreesToRadians(60); // 0 = horizontal
    public static final Translation3d SHOOTER_POS = new Translation3d(0, Units.inchesToMeters(-9.5), 0.51) // forward, left, up
        .rotateBy(new Rotation3d(0, 0, -ANGLE_OFFSET));

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final double BLUE_LONG_SHOOT_X = 2.54;
    public static final double RED_LONG_SHOOT_X = 14;

    public static final double HUB_HEIGHT = 1.829;

    public static final Hub
        BLUE_HUB = new Hub(new Translation2d( 4.626, 4.035), Alliance.Blue),
        RED_HUB  = new Hub(new Translation2d(11.915, 4.035), Alliance.Red);

    public static class Hub {

        public final Translation2d pos;
        public final Alliance alliance;

        private Hub(Translation2d pos, Alliance alliance) {
            this.pos = pos;
            this.alliance = alliance;
        }

        public boolean isOwn() {
            return alliance == DriverStation.getAlliance().orElse(null);
        }

        public boolean isInRange(Translation2d pos) {
            return alliance == Alliance.Red
                ? pos.getX() > this.pos.getX()
                : pos.getX() < this.pos.getX();
        }

    }

    public static Hub getOwnHub() {
        return RED_HUB.isOwn() ? RED_HUB : BLUE_HUB;
    }

    /// IMPL

    private LightSubsystem.ProgressBar velocityDisplay;

    private final Map<CustomTalonFX, PIDController> pid = new HashMap<>();

    public ShooterSubsystem(CustomTalonFX... motors) {
        super(motors, 7);

        for (var motor : motors) {
            motor.setMotorBrake(false);

            pid.put(motor, new PIDController(kP, kI, kD));
        }
    }

    public Command c_basicShoot() {
        return c_forward(true);
    }

    public Command c_controlVelocity(DoubleSupplier getVelocity) {
        if (velocityDisplay == null) {
            // all subsystems should be initialized by now
            velocityDisplay = Component.lights.new ProgressBar(
                new int[] { 255, 0, 0 },
                new boolean[] { true, false, true }
            );
        }

        return runEnd(() -> {
            double vel = getVelocity.getAsDouble();
            double ff = kS * Math.signum(vel) + kV * vel;

            boolean first = true;
            for (var entry : pid.entrySet()) {
                CustomTalonFX motor = entry.getKey();
                PIDController pid = entry.getValue();

                double currentVel = motor.getVelocity().getValueAsDouble() * (motor.getInverted() ? -1 : 1);
                if (first) {
                    velocityDisplay.setProgress(Math.pow(currentVel / vel, 4));
                    Logger.recordOutput("Shooter/RealVelocity", currentVel);
                    first = false;
                }

                double voltage = pid.calculate(currentVel, vel) + ff;
                motor.setVoltage(Util.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE));
            }
        }, this::stop);
    }

    /// COMMANDS

    public boolean canShoot() {
        return getOwnHub().isInRange(Component.chassis.getPositionEstimate());
    }

    public Command c_smartShoot() {
        return c_controlVelocity(() -> calcShooterVelocity(getOwnHub().pos)).andThen(this::stop);
    }

    public static Command c_smartShootAlign() {
        return Component.chassis.c_rotateTo(() -> calcRobotAngle(ShooterSubsystem.getOwnHub().pos), true);
    }

    public Command c_longShoot() {
        return c_controlVelocity(ShooterSubsystem::calcLongShootVelocity).andThen(this::stop);
    }

    public Command c_longShootAlign() {
        return Component.chassis.c_rotateTo(() -> Robot.isRedAlliance() ? 0 : 0.5, true);
    }

    /// DISTANCE MATH

    private static final double tanA = Math.tan(SHOOTER_ANGLE), secA = 1 / Math.cos(SHOOTER_ANGLE);

    public static double getShooterVelocityForDistance(double dist) {
        Logger.recordOutput("Shooter/GoalDistance", dist);

        double dz = HUB_HEIGHT - SHOOTER_POS.getZ();

        double det = dist * tanA - dz;
        if (det <= 0) return 100; // will eventually be clamped to MAX_VOLTAGE

        double ballVel = dist * secA * Math.sqrt(GRAVITY / (2 * det));
        double shooterVel = ballVel / FLYWHEEL_CIRC * VELOCITY_MULT;

        Logger.recordOutput("Shooter/GoalVelocity", shooterVel);
        return shooterVel;
    }

    private static double calcShooterVelocity(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPositionEstimate();
        Translation2d dist = pos.minus(robotPos);

        double dx = dist.getNorm() - SHOOTER_POS.getX();

        if (ACCOUNT_FOR_ROBOT_VEL) {
            Translation2d robotVel = Component.chassis.getVelocity();
            double vx = robotVel.dot(dist.div(dist.getNorm()));
            dx -= vx * AIRTIME_ESTIMATE;
        }

        return getShooterVelocityForDistance(dx);
    }

    private static double calcLongShootVelocity() {
        Translation2d robotPos = Component.chassis.getPositionEstimate();
        double target = Robot.isRedAlliance() ? RED_LONG_SHOOT_X : BLUE_LONG_SHOOT_X;
        double dx = Math.abs(robotPos.getX() - target);

        return getShooterVelocityForDistance(dx);
    }

    /// ALIGNMENT MATH

    private static double calcRobotAngle(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPositionEstimate();
        Translation2d dist = pos.minus(robotPos);

        if (ACCOUNT_FOR_ROBOT_VEL) {
            Translation2d robotVel = Component.chassis.getVelocity();
            dist = dist.minus(robotVel.times(AIRTIME_ESTIMATE));
        }

        double angle = Math.atan2(dist.getY(), dist.getX());
        // account for the fact that the shooter is not aligned with the center of the robot
        double offset = Math.asin(-SHOOTER_POS.getY() / dist.getNorm());

        return Units.radiansToRotations(angle + offset - ANGLE_OFFSET);
    }

}
