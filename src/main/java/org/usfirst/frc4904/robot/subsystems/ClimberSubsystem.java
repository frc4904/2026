package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.custom.sensors.LinearDutyCycleEncoder;
import org.usfirst.frc4904.standard.util.Util;

public class ClimberSubsystem extends MotorSubsystem {

    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MAX_VOLTAGE = 8;

    private static final double MIN_HEIGHT = 0.93; // 0.623;
    private static final double MAX_HEIGHT = 2.8;

    private final LinearDutyCycleEncoder encoder;
   //"@param" adds description to the constructor...hover over below to see
   /**
    * @param motor :it's the motor
    * @param encoder :there is only one encoder, what do you think it's for?
    */
    public ClimberSubsystem(SmartMotorController motor, LinearDutyCycleEncoder encoder) {
        super(motor, 3);

        this.encoder = encoder;
    }

    public Command c_up() {
        return c_forward(true);
    }

    public Command c_down() {
        return c_backward(true);
    }

    public double getHeight() {
        return encoder.get();
    }

    public Command c_gotoUp() {
        return c_gotoHeight(MAX_HEIGHT);
    }

    public Command c_gotoDown() {
        return c_gotoHeight(MIN_HEIGHT);
    }

    public Command c_gotoHeight(double height) {
        var pid = new PIDController(kP, kI, kD);

        return c_controlVoltage(() -> {
            double effort = pid.calculate(getHeight(), height);
            return Util.clamp(effort, -MAX_VOLTAGE, MAX_VOLTAGE);
        }, true);
    }

    @Override
    public void setVoltage(double voltage) {
        setVoltage(voltage, false);
    }

    public void setVoltage(double voltage, boolean bypassSoftwareStop) {
        if (
            !bypassSoftwareStop && (
                (this.getHeight() >= MAX_HEIGHT && voltage > 0) ||
                (this.getHeight() <= MIN_HEIGHT && voltage < 0)
            )
        ) {
            voltage = 0;
        }
        super.setVoltage(voltage);
    }
}
