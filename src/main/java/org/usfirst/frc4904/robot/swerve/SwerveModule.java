package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.custom.sensors.CustomDutyCycleEncoder;
import org.usfirst.frc4904.standard.util.Util;

import static org.usfirst.frc4904.robot.swerve.SwerveConstants.*;

public class SwerveModule implements Sendable {

    final String name;
    final Translation2d position;

    private final DriveController drive;
    private final RotationController rotation;

    private double magnitude;
    private double theta;

    public SwerveModule(
        String name,
        CustomTalonFX driveMotor,
        SmartMotorController rotMotor,
        CustomDutyCycleEncoder rotEncoder,
        Translation2d position
    ) {
        this.name = name;
        // TODO maybe remove normalization and make it the caller's responsibility to pass the position in meters
        this.position = position.times(ROBOT_DIAGONAL / (2 * position.getNorm()));

        Translation2d direction = position.rotateBy(Rotation2d.kCCW_90deg);

        drive = driveMotor != null ? new DriveController(name, driveMotor) : null;
        rotation = new RotationController(name, rotMotor, rotEncoder, direction);

        theta = rotation.getRotation();

        SmartDashboard.putData("swerve/" + name, this);
    }

    SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            drive.getDistance(),
            Rotation2d.fromRotations(rotation.getRotation())
        );
    }

    SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            drive.getVelocity(),
            Rotation2d.fromRotations(rotation.getRotation())
        );
    }

    Translation2d rotToTranslation(double theta) {
        return rotation.toTranslation(theta);
    }

    void zero() {
        rotation.zero();
    }

    void flipZero() {
        rotation.flipZero();
    }

    void setMotorBrake(boolean brake) {
        drive.motor().setMotorBrake(brake);
        rotation.motor.setMotorBrake(brake);
    }

    void moveTo(double magnitude, double theta) {
        this.magnitude = magnitude;
        if (magnitude > 0) this.theta = theta;
    }

    void periodic() {
        double angleDist = rotation.rotateToward(theta);
        if (drive != null) drive.setMagnitude(magnitude * angleDist);
    }

    /// SMART DASHBOARD

    void addSendableProps(SendableBuilder builder) {
        builder.addDoubleProperty(name + " Angle", () -> theta, null);
        builder.addDoubleProperty(name + " Velocity", () -> magnitude, null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");
        builder.addDoubleProperty("angle", rotation::getRotation, null);
        builder.addDoubleProperty("delta", () -> {
            double delta = theta - rotation.getRotation();
            return MathUtil.inputModulus(delta, -0.25, 0.25);
        }, null);
        builder.addDoubleProperty("zero", rotation.encoder::getResetOffset, rotation.encoder::setResetOffset);
    }
}

record DriveController(String name, CustomTalonFX motor) {
    public void setMagnitude(double magnitude) {
        motor.set(magnitude / LIN_SPEED);
    }

    double getDistance() {
        return motor.getPosition().getValueAsDouble() / DRIVE_GEAR_RATIO * WHEEL_CIRC;
    }

    double getVelocity() {
        return motor.getVelocity().getValueAsDouble() / DRIVE_GEAR_RATIO * WHEEL_CIRC;
    }
}

class RotationController {
    private static final double kP = 15, kI = 0, kD = 0;

    private static final double MAX_VOLTAGE = 4;

    final String name;

    final SmartMotorController motor;
    final CustomDutyCycleEncoder encoder;

    private final Translation2d direction;

    private final PIDController pid;

    public RotationController(
        String name,
        SmartMotorController motor,
        CustomDutyCycleEncoder encoder,
        Translation2d direction
    ) {
        this.name = name;
        this.motor = motor;
        this.encoder = encoder;

        this.direction = direction.div(direction.getNorm());

        this.pid = new PIDController(kP, kI, kD);
        // encoder readings are from 0-1 but opposite angles are equivalent
        // since we can just run the wheels backwards
        this.pid.enableContinuousInput(0, 0.5);
    }

    Translation2d toTranslation(double theta) {
        return direction.times(theta);
    }

    void zero() {
        encoder.reset();
    }

    void flipZero() {
        encoder.flip();
    }

    double getRotation() {
        return -encoder.get();
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * @return Similarity between current and target rotation.
     *         Effectively a dot product: 1 = same angle, -1 = opposite, 0 = perpendicular.
     */
    public double rotateToward(double theta) {
        double current = getRotation();
        double voltage = pid.calculate(current, theta);
        setVoltage(Util.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE));

        return Math.cos(Units.rotationsToRadians(theta - current));
    }
}
