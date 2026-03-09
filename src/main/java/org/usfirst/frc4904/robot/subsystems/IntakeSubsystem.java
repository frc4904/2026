package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class IntakeSubsystem extends MotorSubsystem {

    public static final double kP = 4;
    public static final double kI = 4; // dubious
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 4;
    public static final double kA = 0;
    public static final double kG = -0.5; // negative b/c positive voltage is down

    public static final double RETRACT_ANGLE = 0.94;
    public static final double EXTEND_ANGLE = 0.175;
    public static final double HORIZONTAL = 0.18;

    public static final double MAX_VEL = 1;
    public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second

    private final SmartMotorController angleMotor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward feedforward;

    public IntakeSubsystem(
        SmartMotorController angleMotor,
        SmartMotorController rollerMotor,
        DutyCycleEncoder encoder
    ) {
        super(rollerMotor, 6);

        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.feedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    public double getAngle() {
        return encoder.get();
    }

    public Command c_intake() {
        return c_forward(true);
    }

    // private final Subsystem angleMotorRequirement = new SubsystemBase("intake vertical motor") {};
    private final Subsystem angleMotorRequirement = Component.INTAKE_DO_NOT_USE_OR_YOU_WILL_BE_FIRED;

    public Command c_extend() {
        return c_gotoAngle(EXTEND_ANGLE);
    }

    public Command c_retract() {
        return c_gotoAngle(RETRACT_ANGLE);
    }

    public Command c_gotoAngle(double angle) {
        ezControl control = new ezControl(
            kP, kI, kD,
            (pos, vel) -> feedforward.calculate(
                Units.rotationsToRadians(getAngle() - HORIZONTAL),
                vel
            )
        );
        control.pid.enableContinuousInput(0, 1);
        var constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);

        return new ezMotion(
            control,
            this::getAngle,
            angleMotor::setVoltage,
            angle,
            1,
            constraints,
            angleMotorRequirement
        ).finallyDo(this::stop);
    }
}
