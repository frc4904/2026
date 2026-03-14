package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.util.Logging;

import java.util.function.DoubleSupplier;

public class MotorSubsystem extends SubsystemBase {

    public final SmartMotorController[] motors;

    protected double forwardVoltage;
    protected double backwardVoltage;

    /**
     * Control multiple motors with one subsystem. This constructor only controls one motor.
     *
     * @param motor the motor that the subsystem controls
     * @param voltage voltage when motors are running
     */
    public MotorSubsystem(SmartMotorController motor, double voltage) {
        this(new SmartMotorController[] { motor }, voltage, voltage);
    }

    /**
     * Control multiple motors with one subsystem. This constructor only controls one motor.
     *
     * @param motor the motor that the subsystem controls
     * @param forwardVoltage voltage when the motor is running forwards
     * @param backwardVoltage voltage when the motor is running backwards - should be POSITIVE (is negated later)
     */
    public MotorSubsystem(SmartMotorController motor, double forwardVoltage, double backwardVoltage) {
        this(new SmartMotorController[] { motor }, forwardVoltage, backwardVoltage);
    }

    /**
     * Control multiple motors with one subsystem. For example, to have two motors, use:
     * <pre>{@code
     *     new MotorSubsystem(
     *         new SmartMotorController[] { motor1, motor2 },
     *         voltage
     *     )
     * }</pre>
     *
     * @param motors the motors that the subsystem controls
     * @param voltage voltage when motors are running
     */
    public MotorSubsystem(SmartMotorController[] motors, double voltage) {
        this(motors, voltage, voltage);
    }

    /**
     * Control multiple motors with one subsystem. For example, to have two motors, use:
     * <pre>{@code
     *     new MotorSubsystem(
     *         new SmartMotorController[] { motor1, motor2 },
     *         forwardVoltage,
     *         backwardVoltage
     *     )
     * }</pre>
     *
     * @param motors the motors that the subsystem controls
     * @param forwardVoltage voltage when motors are running forwards
     * @param backwardVoltage voltage when motors are running backwards - should be POSITIVE (is negated later)
     */
    public MotorSubsystem(SmartMotorController[] motors, double forwardVoltage, double backwardVoltage) {
        this.motors = motors;
        this.forwardVoltage = forwardVoltage;
        this.backwardVoltage = backwardVoltage;

        setMotorBrake(true);
    }

    public void setVoltage(double voltage) {
        for (var motor : motors) motor.setVoltage(voltage);
    }

    public void stop() {
        // presumably the same as setVoltage(0)
        for (var motor : motors) motor.stopMotor();
    }

    public void setMotorBrake(boolean brake) {
        for (var motor : motors) motor.setMotorBrake(brake);
    }

    public Command c_controlVoltage(DoubleSupplier getVoltage, boolean stopOnEnd) {
        return runEnd(
            () -> setVoltage(getVoltage.getAsDouble()),
            stopOnEnd ? this::stop : () -> {}
        );
    }

    public Command c_holdVoltage(double voltage, boolean stopOnEnd) {
        return c_controlVoltage(() -> voltage, stopOnEnd);
    }

    public Command c_forward(boolean stopOnEnd) {
        return c_holdVoltage(forwardVoltage, stopOnEnd);
    }

    public Command c_backward(boolean stopOnEnd) {
        return c_holdVoltage(-backwardVoltage, stopOnEnd);
    }

    public Command c_stop() {
        return runOnce(this::stop);
    }

    /**
     * Temporary command for tuning {@code kG} PID constant. For example:
     * <pre>{@code
     * joystick.button7.whileTrue(
     *     Component.arm.c_DEBUG_tunePIDkG(() -> joystick.getAxis(Axis.SLIDER))
     * );
     * }</pre>
     * Run this command and move the slider until the mechanism is receiving
     * just enough voltage to counter gravity but not enough to move upwards.
     * For {@link ArmFeedforward}, the arm should be at the horizontal angle
     * (if not, {@code kG} should be set to {@code FINAL_VOLTAGE / Math.cos(radiansFromHorizontal)}).
     */
    public Command c_DEBUG_tunePIDkG(DoubleSupplier getVoltage) {
        return c_controlVoltage(() -> {
            double voltage = getVoltage.getAsDouble();
            Logging.logWithDelay("debug kG tuning - voltage", 0.1, voltage);
            return voltage;
        }, true).finallyDo(
            () -> System.out.println("FINAL VOLTAGE: " + getVoltage.getAsDouble())
        );
    }

}
