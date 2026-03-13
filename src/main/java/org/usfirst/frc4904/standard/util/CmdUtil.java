package org.usfirst.frc4904.standard.util;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import org.usfirst.frc4904.standard.commands.NoOp;

import java.util.Collections;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

public final class CmdUtil {
    private CmdUtil() {
        throw new UnsupportedOperationException("Cannot instantiate utility class.");
    }

    private static CommandScheduler scheduler() {
        return CommandScheduler.getInstance();
    }

    public static void schedule(Command... commands) {
        scheduler().schedule(commands);
    }

    /**
     * Cancels all commands that conflict with the given subsystems
     */
    public static void cancelConflicting(Subsystem... requirements) {
        for (var subsystem : requirements) {
            var cmd = subsystem.getCurrentCommand();
            if (cmd != null) cmd.cancel();
        }
    }

    /**
     * <em>Attempts</em> to cancel all commands that conflict with the given subsystems.
     * Commands will only be canceled if none of the conflicting commands have their
     * {@link Command#getInterruptionBehavior() interruption behavior} set to
     * {@link InterruptionBehavior#kCancelIncoming kCancelIncoming}.
     * <p>
     * For example:
     * <pre>{@code
     * if (CmdUtil.tryCancelConflicting(Component.elevator)) {
     *     Component.elevator.setVoltage(2); // not command-based
     * }
     * }</pre>
     * @return {@code true} if conflicting commands were canceled
     */
    public static boolean tryCancelConflicting(Subsystem... requirements) {
        for (var subsystem : requirements) {
            var cmd = subsystem.getCurrentCommand();
            if (cmd.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming) {
                return false;
            }
        }

        cancelConflicting(requirements);
        return true;
    }

    /**
     * Replace {@code null}s with a {@link NoOp no-op} command instead
     * @param cmd Command or {@code null}
     * @return Non-null command
     */
    public static Command nonNull(Command cmd) {
        return cmd == null ? new NoOp() : cmd;
    }

    /**
     * Makes commands finish instantly (but keep running) by scheduling
     * them instead of running them directly.
     */
    public static Command asInstant(Command... commands) {
        return new InstantCommand(() -> schedule(commands));
    }

    public static Command delayed(double delaySeconds, Command command) {
        return new WaitCommand(delaySeconds).andThen(command);
    }

    /**
     * Useful for creating complicated functional commands with multiple lambdas that
     * reference the same state, and where the state needs to be updated every iteration.
     * <p>
     * For example:
     * <pre>{@code
     * return CmdUtil.<Pose2d>withState(
     *     this::getNextPose,
     *     state -> new ParallelCommandGroup(
     *         c_moveTo(() -> state.get().getTranslation())),
     *         c_rotateTo(() -> state.get().getRotation())
     *     )
     * )
     * }</pre>
     * @param getState Supplier of the current state
     * @param getCommand Lambda that takes in a state supplier for usage in the functional command(s),
     *                   and returns a command. Use {@code state.get()} to retrieve the current state
     *                   within this lambda. Calling {@code state.get()} does <em>not</em> recompute
     *                   the state - it returns the state that was already computed this iteration
     * @param addedRequirements Additional requirements to add
     * @return The constructed command
     * @param <T> The type of the state
     */
    public static <T> Command withState(
        Supplier<T> getState,
        Function<Supplier<T>, Command> getCommand,
        Subsystem... addedRequirements
    ) {
        var state = new Supplier<T>() {
            T val;
            @Override public T get() { return val; }
        };

        Command cmd = getCommand.apply(state);
        Set<Subsystem> requirements = cmd.getRequirements();
        Collections.addAll(requirements, addedRequirements);

        return new WrapperCommand(cmd) {
            @Override
            public void execute() {
                state.val = getState.get();
                super.execute();
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

}
