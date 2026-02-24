package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.SwitchingIfElseCommand;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class ShooterSubsystem extends MotorSubsystem {

    /// MEASUREMENTS

    public static final double GRAVITY = 9.8;

    // TODO get measurements
    public static final double SHOOTER_ANGLE = Units.degreesToRadians(0); // 0 = horizontal
    public static final Translation3d SHOOTER_POS = new Translation3d(0.1, -0.2, 0.3); // forward, left, up

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final double HUB_HEIGHT = 1.829;

    public static final Hub
        RED_HUB  = new Hub(new Translation2d( 4.626, 4.035), Alliance.Red,  pos -> pos.getX() < 4.626),
        BLUE_HUB = new Hub(new Translation2d(11.915, 4.035), Alliance.Blue, pos -> pos.getX() > 11.915);

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

    /// TUNING

    // TODO tune
    private static final double MAX_VEL = 10;

    private static final Slot0Configs pidfConfig = new Slot0Configs();
    static {
        // TODO tune
        pidfConfig.kP = 1;
        pidfConfig.kI = 0;
        pidfConfig.kD = 0;
        pidfConfig.kS = 0;
        pidfConfig.kV = 0;
        pidfConfig.kA = 0;
        pidfConfig.kG = 0;
    }

    /// IMPL

    public ShooterSubsystem(CustomTalonFX motor1, CustomTalonFX motor2) {
        super(
            new SmartMotorController[] { motor1, motor2 },
            4
        );

        motor1.getConfigurator().apply(pidfConfig);
        motor2.getConfigurator().apply(pidfConfig);
    }

    public Command c_basicShoot() {
        return c_forward(true);
    }

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Command c_controlVelocity(DoubleSupplier getVelocity) {
        return runEnd(() -> {
            velocityRequest.withVelocity(getVelocity.getAsDouble());
            for (var motor : motors) {
                ((CustomTalonFX) motor).setControl(velocityRequest);
            }
        }, this::stop);
    }

    public Command c_smartShoot() {
        // TODO prefer closest instead? (maybe helpful for testing)
        Hub hub = getOwnHub();

        return new SwitchingIfElseCommand(
            new ParallelCommandGroup(
                c_controlVelocity(() -> calcShooterVelocity(hub.pos)),
                Component.chassis.c_rotateTo(() -> calcRobotAngle(hub.pos))
            ),
            c_stop(), // TODO fun lights, elastic notif?, etc. on fail
            () -> hub.isInRange(Component.chassis.getPoseEstimate().getTranslation()),
            () -> hasSufficientDistance(hub.pos)
        ).andThen(this::stop);
    }

    private static final double
        // sin2A = Math.sin(2 * SHOOTER_ANGLE),
        tanA = Math.tan(SHOOTER_ANGLE),
        secA = 1 / Math.cos(SHOOTER_ANGLE);

    private double calcShooterVelocity(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPoseEstimate().getTranslation();

        double dx = pos.getDistance(robotPos) - SHOOTER_POS.getX();
        double dz = HUB_HEIGHT - SHOOTER_POS.getZ();

        double determinant = dx * tanA - dz;
        if (determinant <= 0) return MAX_VEL;

        double vel = dx * secA * Math.sqrt(GRAVITY / (2 * determinant));
        return Math.min(vel, MAX_VEL);
    }

    private boolean hasSufficientDistance(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPoseEstimate().getTranslation();

        double dx = pos.getDistance(robotPos) - SHOOTER_POS.getX();
        // TODO tune
        return dx > 1; // just tune it - the math is not particularly accurate in practice anyway

        // double dz = HUB_HEIGHT - SHOOTER_POS.getZ();
        // double apex = (dx * dx * sin2A * secA * secA) / (4 * (dx * tanA - dz));
        // return dx > apex;
    }

    private double calcRobotAngle(Translation2d pos) {
        Translation2d robotPos = Component.chassis.getPoseEstimate().getTranslation();
        Translation2d dist = pos.minus(robotPos);

        double angle = Math.atan2(dist.getY(), dist.getX());
        // account for the fact that the shooter is not aligned with the center of the robot
        double offset = Math.asin(-SHOOTER_POS.getY() / dist.getNorm());

        return Units.radiansToRotations(angle + offset);
    }

}
