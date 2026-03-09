package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.custom.sensors.LinearDutyCycleEncoder;

public class ClimberSubsystem extends MotorSubsystem {

    // TODO change all these
    public static final double kS = 1;
    public static final double kV = 2;
    public static final double kA = 0.4;
    public static final double kG = 0.2;

    public static final double kP = 6;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MAX_VEL = 8;
    public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second

    private static final double MAX_HEIGHT = 123456789;
    private static final double MIN_HEIGHT = 0;

    private final ElevatorFeedforward feedforward;
    private final LinearDutyCycleEncoder encoder;

    public ClimberSubsystem(SmartMotorController motor, LinearDutyCycleEncoder encoder) {
        super(motor, 3);

        this.encoder = encoder;
        this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
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

    public Command c_gotoHeight(double height) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (pos, vel) -> feedforward.calculate(vel)
        );
        var constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);

        return new ezMotion(
            controller,
            this::getHeight,
            this::setVoltage,
            height,
            constraints,
            this
        ).finallyDo(this::stop);
    }

    @Override
    public void setVoltage(double voltage) {
        setVoltage(voltage, true);
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
