package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ezMotion extends Command {

    public final ezControl control;

    private final DoubleSupplier getCurrent;
    private final DoubleConsumer processValue;
    private final Supplier<? extends SetpointSupplier> setpointDealerDealer;

    private SetpointSupplier setpointDealer;
    private double initialTimestamp;

    /**
     * See {@link #ezMotion(ezControl, DoubleSupplier, DoubleConsumer, SetpointSupplier, Subsystem...) here} for other parameters.
     *
     * @param setpointDealerDealer Supplier of setpoint suppliers.
     *                             A new setpoint supplier will be gotten each time this command runs.
     */
    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        Supplier<? extends SetpointSupplier> setpointDealerDealer,
        Subsystem... requirements
    ) {
        this.control = control;
        this.getCurrent = getCurrent;
        this.processValue = processValue;
        this.setpointDealerDealer = setpointDealerDealer;

        addRequirements(requirements);
    }

    /**
     * @param control PID and feedforward config. Should not be shared by multiple
     *                ezMotion commands if they can run at the <em>same time</em>.
     * @param getCurrent Supplier that get the current position/angle of the mechanism.
     * @param processValue Consumer that applies calculated values (usually voltages) to the mechanism.
     * @param setpointDealer Supplier of setpoints that the feedforward will target.
     * @param requirements Requirements for this command.
     */
    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        SetpointSupplier setpointDealer,
        Subsystem... requirements
    ) {
        this(control, getCurrent, processValue, () -> setpointDealer, requirements);
    }

    /**
     * See {@link #ezMotion(ezControl, DoubleSupplier, DoubleConsumer, SetpointSupplier, Subsystem...) here} for other parameters.
     *
     * @param goal The goal position. Each time the command starts,a {@link TrapezoidProfile}
     *             will be calculated from the current position to this goal.
     * @param constraints The {@link Constraints constraints} for the above profile.
     */
    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        double goal,
        Constraints constraints,
        Subsystem... requirements
    ) {
        this(
            control,
            getCurrent,
            processValue,
            () -> {
                var profile = new TrapezoidProfile(constraints);
                var startState = new TrapezoidProfile.State(getCurrent.getAsDouble(), 0);
                var goalState = new TrapezoidProfile.State(goal, 0);
                return (elapsed) -> profile.calculate(elapsed, startState, goalState);
            },
            requirements
        );
    }

    /**
     * See {@link #ezMotion(ezControl, DoubleSupplier, DoubleConsumer, double, Constraints, Subsystem...) here} for other parameters.
     *
     * @param continuousRange The range of motion for a rotational mechanism before it wraps around to the same position.
     *                        For example, if {@code 1}, then positions {@code 0 == 1}, {@code 0.5 == 1.5}, etc.
     *                        This is used to calculate the trapezoid profile, <em>not</em> for continuous PID.
     *                        Make sure to also use {@link PIDController#enableContinuousInput(double, double)} when constructing the {@link ezControl}.
     */
    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        double goal,
        double continuousRange,
        Constraints constraints,
        Subsystem... requirements
    ) {
        this(
            control,
            getCurrent,
            processValue,
            () -> {
                double current = getCurrent.getAsDouble();
                double wrappedGoal = MathUtil.inputModulus(goal, current - continuousRange / 2, current + continuousRange / 2);

                var profile = new TrapezoidProfile(constraints);
                var startState = new TrapezoidProfile.State(current, 0);
                var goalState = new TrapezoidProfile.State(wrappedGoal, 0);
                return (elapsed) -> profile.calculate(elapsed, startState, goalState);
            },
            requirements
        );
    }

    public double getElapsedTime() {
        return Timer.getFPGATimestamp() - initialTimestamp;
    }

    @Override
    public void initialize() {
        initialTimestamp = Timer.getFPGATimestamp();
        setpointDealer = setpointDealerDealer.get();
        control.reset();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State setpoint = setpointDealer.get(getElapsedTime());
        double controlEffort = control.calculate(getCurrent.getAsDouble(), setpoint);
        processValue.accept(controlEffort);
    }

    @FunctionalInterface
    public interface SetpointSupplier {
        TrapezoidProfile.State get(double elapsed);
    }

    @FunctionalInterface
    public interface FeedForward {
        FeedForward NOOP = (pos, vel) -> 0;

        double calculate(double position, double velocity);
    }
}
