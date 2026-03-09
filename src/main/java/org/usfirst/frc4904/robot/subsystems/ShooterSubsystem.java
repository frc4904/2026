package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.util.Util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class ShooterSubsystem extends MotorSubsystem {

    /// TUNING

    private static final boolean ACCOUNT_FOR_ROBOT_VEL = false;

    // approximate amount of seconds the fuel spends in the air
    // used to account for robot velocity
    private static final double AIRTIME_ESTIMATE = 1;

    // multiplier to get from target fuel velocity to target flywheel velocity
    // in theory, this would be ~2 to account for the rotation of the fuel as it leaves the shooter
    private static final double VELOCITY_MULT = 6;

    // hardcoded offset between the robot angle and the exit angle of the fuel
    // positive means that the fuel exits the robot to the left/counterclockwise of the expected angle
    private static final double ANGLE_OFFSET = Units.degreesToRadians(0);

    // TODO tune
    private static final double MAX_VEL = 8;

    private static final double kP=0 , kI = 0, kD = 0, kS = 0, kV = 0.7
    ;

    /// MEASUREMENTS

    public static final double GRAVITY = 9.8;

    // TODO get measurements
    public static final double FLYWHEEL_CIRC = Units.inchesToMeters(4 * Math.PI);
    public static final double SHOOTER_ANGLE = Units.degreesToRadians(60); // 0 = horizontal
    public static final Translation3d SHOOTER_POS = new Translation3d(0, -0.22, 0.51) // forward, left, up
        .rotateBy(new Rotation3d(0, 0, -ANGLE_OFFSET));

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final double HUB_HEIGHT = 1.829;

    public static final Hub
        BLUE_HUB = new Hub(new Translation2d( 4.626, 4.035), Alliance.Blue, pos -> pos.getX() < 4.626),
        RED_HUB  = new Hub(new Translation2d(11.915, 4.035), Alliance.Red,  pos -> pos.getX() > 11.915);

    public static class Hub {

        public final Translation2d pos;
        public final Alliance alliance;

        private final Predicate<Translation2d> inRange;

        private Hub(Translation2d pos, Alliance alliance, Predicate<Translation2d> inRange) {
            this.pos = pos;
            this.alliance = alliance;
            this.inRange = inRange;
        }

        public boolean isOwn() {
            return alliance == DriverStation.getAlliance().orElse(null);
        }

        public boolean isInRange(Translation2d pos) {
            return inRange.test(pos);
        }

    }

    public static Hub getOwnHub() {
        return RED_HUB.isOwn() ? RED_HUB : BLUE_HUB;
    }

    /// IMPL

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
        return runEnd(() -> {
            double vel = Util.clamp(getVelocity.getAsDouble(), -MAX_VEL, MAX_VEL);

            double ff = kS * Math.signum(vel) + kV * vel;

            for (var entry : pid.entrySet()) {
                CustomTalonFX motor = entry.getKey();
                PIDController pid = entry.getValue();

                double currentVel = motor.getVelocity().getValueAsDouble() * (motor.getInverted() ? -1 : 1);
                motor.setVoltage(pid.calculate(currentVel, vel) + ff);
            }
        }, this::stop);
    }

    /// SHOOTER MATH

    public boolean canShoot() {
        return getOwnHub().isInRange(Component.chassis.getPositionEstimate());
    }

    public Command c_smartShoot() {
        return c_controlVelocity(() -> calcShooterVelocity(getOwnHub().pos)).andThen(this::stop);
    }

    private static final double tanA = Math.tan(SHOOTER_ANGLE), secA = 1 / Math.cos(SHOOTER_ANGLE);

    public static double getShooterVelocityForDistance(double dist) {
        double dz = HUB_HEIGHT - SHOOTER_POS.getZ();

        double determinant = dist * tanA - dz;
        if (determinant <= 0) return MAX_VEL;

        double vel = dist * secA * Math.sqrt(GRAVITY / (2 * determinant));
        return vel * VELOCITY_MULT / FLYWHEEL_CIRC;
    }

    public static double calcRobotAngle(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPositionEstimate();
        Translation2d dist = pos.minus(robotPos);

        if (ACCOUNT_FOR_ROBOT_VEL) {
            Translation2d robotVel = Component.chassis.getVelocityEstimate();
            dist = dist.minus(robotVel.times(AIRTIME_ESTIMATE));
        }

        double angle = Math.atan2(dist.getY(), dist.getX());
        // account for the fact that the shooter is not aligned with the center of the robot
        double offset = Math.asin(-SHOOTER_POS.getY() / dist.getNorm());

        return Units.radiansToRotations(angle + offset - ANGLE_OFFSET);
    }

    private static double calcShooterVelocity(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPositionEstimate();
        Translation2d dist = pos.minus(robotPos);

        double dx = dist.getNorm() - SHOOTER_POS.getX();

        if (ACCOUNT_FOR_ROBOT_VEL) {
            Translation2d robotVel = Component.chassis.getVelocityEstimate();
            double vx = robotVel.dot(dist.div(dist.getNorm()));
            dx -= vx * AIRTIME_ESTIMATE;
        }

        SmartDashboard.putNumber("cheese distance", dx);
        return getShooterVelocityForDistance(dx);
    }

}
