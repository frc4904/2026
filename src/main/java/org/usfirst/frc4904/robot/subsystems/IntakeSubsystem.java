package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.util.Util;

public class IntakeSubsystem extends MotorSubsystem {

    public static final double kP = 4;
    public static final double kI = 1.3;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 4;
    public static final double kA = 0;
    public static final double kG = -0.3; // negative b/c positive voltage is down

    public static final double RETRACT_ANGLE = 0.94;
    public static final double EXTEND_ANGLE = 0.18;
    public static final double HORIZONTAL = 0.175;

    public static final double MAX_VEL = 0.7;
    public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second

    public static final double WOBBLE_SPEED = 1.5; // Hz
    public static final double WOBBLE_DISTANCE = Units.degreesToRotations(45);

    private final SmartMotorController angleMotor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward feedforward;

    public IntakeSubsystem(
        SmartMotorController angleMotor,
        SmartMotorController rollerMotor,
        DutyCycleEncoder encoder
    ) {
        super(rollerMotor, SmartDashboard.getNumber("intake/roller volts", 6));

        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.feedforward = new ArmFeedforward(kS, kG, kV, kA);

        SmartDashboard.putData("intake", this);
    }

    public double getAngle() {
        return encoder.get();
    }

    public Command c_intake() {
        return c_forward(true);
    }

    // private final Subsystem angleMotorRequirement = new SubsystemBase("intake vertical motor") {};
    private final Subsystem angleMotorRequirement = Component.INTAKE_DO_NOT_USE_OR_YOU_WILL_BE_FIRED;

    private boolean wobble = false;

    // c_wobble() only has an effect if a c_gotoAngle() command is running simultaneously,
    // which should always be true after the first intake up/down.
    public Command c_wobble() {
        // no requirement since this should run in parallel with c_gotoAngle()
        return new StartEndCommand(() -> wobble = true, () -> wobble = false);
    }

    public Command c_extend() {
        return c_gotoAngle(EXTEND_ANGLE, -WOBBLE_DISTANCE);
    }

    public Command c_retract() {
        return c_gotoAngle(RETRACT_ANGLE, WOBBLE_DISTANCE);
    }

    public Command c_gotoAngle(double angle, double wobbleDistance) {
        ezControl control = new ezControl(
            kP, kI, kD,
            (pos, vel) -> feedforward.calculate(
                Units.rotationsToRadians(getAngle() - HORIZONTAL),
                vel
            )
        );
        control.pid.enableContinuousInput(0, 1);
        var profile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));

        return new ezMotion(
            control,
            this::getAngle,
            angleMotor::setVoltage,
            () -> {
                double current = getAngle();
                double wrappedGoal = MathUtil.inputModulus(angle, current - 0.5, current + 0.5);

                var startState = new State(current, 0);
                var goalState = new State(wrappedGoal, 0);

                return (elapsed) -> {
                    State setpoint = profile.calculate(elapsed, startState, goalState);
                    if (!wobble) return setpoint;

                    // increase wobbling the closer we are to the target angle
                    double dist = Math.abs(MathUtil.inputModulus(setpoint.position - angle, -0.5, 0.5));
                    double wobbleMag = Util.transformRange(dist, 0, 0.2, 1, 0);
                    if (wobbleMag <= 0) return setpoint;

                    double t = Timer.getFPGATimestamp();
                    double speed = 2 * Math.PI * WOBBLE_SPEED;
                    double wobbleScale = wobbleMag * wobbleDistance;

                    double wobblePos = (Math.sin(t * speed) + 1) / 2;
                    double wobbleVel = speed * Math.cos(t * speed) / 2; // calculus reference

                    return new State(
                        setpoint.position + wobblePos * wobbleScale,
                        setpoint.velocity + wobbleVel * wobbleScale
                    );
                };
            },
            angleMotorRequirement
        ).finallyDo(this::stop);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "roller volts",
            () -> forwardVoltage,
            (v) -> forwardVoltage = backwardVoltage = v
        );
    }
}
