package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class MotorSubsystem extends SubsystemBase {

    public final SmartMotorController[] motors;

    public final double forwardVoltage;
    public final double backwardVoltage;

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

    public Command c_holdVoltage(double voltage, boolean stopOnEnd) {
        return runEnd(
            () -> setVoltage(voltage),
            stopOnEnd ? this::stop : () -> {}
        );
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

}
