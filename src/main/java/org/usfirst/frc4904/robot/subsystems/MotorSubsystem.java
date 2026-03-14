package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.util.Logging;

import java.util.Arrays;
import java.util.function.Supplier;
import java.util.function.DoubleSupplier;
import java.util.stream.DoubleStream;

public class MotorSubsystem extends SubsystemBase {

    public final SmartMotorController[] motors;

    public final double[] forwardVoltages;
    public final double[] backwardVoltages;

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
        this(
            motors,
            fill(motors.length, forwardVoltage),
            fill(motors.length, backwardVoltage)
        );
    }

    /**
     * TODO
     */
    public MotorSubsystem(SmartMotorController[] motors, double[] forwardVoltages, double[] backwardVoltages) {
        this.motors = motors;
        this.forwardVoltages = forwardVoltages;
        this.backwardVoltages = backwardVoltages;

        setMotorBrake(true);
    }

    public void stop() {
        // presumably the same as setVoltage(0)
        for (var motor : motors) motor.stopMotor();
    }

    public void setMotorBrake(boolean brake) {
        for (var motor : motors) motor.setMotorBrake(brake);
    }

    /// SINGLE

    public void setVoltage(double voltage) {
        setVoltages(fill(motors.length, voltage));
    }

    public Command c_holdVoltage(double voltage, boolean stopOnEnd) {
        return c_controlVoltage(() -> voltage, stopOnEnd);
    }

    public Command c_controlVoltage(DoubleSupplier getVoltage, boolean stopOnEnd) {
        return runEnd(
            () -> setVoltage(getVoltage.getAsDouble()),
            stopOnEnd ? this::stop : () -> {}
        );
    }

    /// MULTIPLE

    public void setVoltages(double[] voltages) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setVoltage(voltages[i]);
        }
    }

    public Command c_holdVoltages(double[] voltages, boolean stopOnEnd) {
        return c_controlVoltages(() -> voltages, stopOnEnd);
    }

    public Command c_controlVoltages(Supplier<double[]> getVoltages, boolean stopOnEnd) {
        return runEnd(
            () -> setVoltages(getVoltages.get()),
            stopOnEnd ? this::stop : () -> {}
        );
    }

    public Command c_forward(boolean stopOnEnd) {
        return c_holdVoltages(forwardVoltages, stopOnEnd);
    }

    public Command c_backward(boolean stopOnEnd) {
        return c_holdVoltages(Arrays.stream(backwardVoltages).map(v -> -v).toArray(), stopOnEnd);
    }

    public Command c_stop() {
        return runOnce(this::stop);
    }

    private static double[] fill(int length, double value) {
        return DoubleStream.generate(() -> value).limit(length).toArray();
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
