package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * An AsyncSequence command is composed of a list of commands that are either
 * <strong>synchronous</strong> (passed as a normal argument) or <strong>asynchronous</strong>
 * (wrapped with {@link #async(Command...) async()}). At any given time when this command
 * is running...
 * <ul>
 *     <li>
 *         One <strong>synchronous</strong> command will be running, unless all sync
 *         commands have finished and some async commands are still running.
 *     </li>
 *     <li>
 *         Any <strong>asynchronous</strong> commands that were earlier in the list
 *         than the current sync command will also be running, unless a later command
 *         (sync or async) has been initialized with conflicting requirements.
 *         {@link InterruptionBehavior Interruption behavior} is ignored within this command.
 *     </li>
 * </ul>
 * For example, when this command runs:
 * <pre>{@code
 * new AsyncSequence(
 *     async(climber.c_up()),
 *     c_autonPartA(),
 *     c_autonPartB(),
 *     climber.c_down()
 * )
 * }</pre>
 * The {@code climber.c_up()} and {@code c_autonPartA()} commands will start at the same time,
 * and the climber up command will continue to run through auton parts A and B. Once
 * {@code climber.c_down()} is reached, the climber up command will be canceled since it has
 * conflicting requirements.
 */
public class AsyncSequence extends Command {

    private static class AsyncWrapper extends Command {
        private final Command[] commands;

        private AsyncWrapper(Command... commands) {
            if (commands.length == 0) {
                throw new IllegalArgumentException("AsyncSequence.async() must be called with at least one command.");
            }

            this.commands = commands;

            for (var cmd : commands) {
                // aggregate requirements so that conflicting commands can be cancelled at once
                var requirements = cmd.getRequirements();
                if (!Collections.disjoint(requirements, getRequirements())) {
                    throw new IllegalArgumentException("Commands passed to AsyncSequence.async() must not have conflicting requirements.");
                }
                addRequirements(requirements);
            }
        }

        @Override
        public void initialize() {
            throw new IllegalStateException("The returned command from AsyncSequence.async() can only be used as an argument to the AsyncSequence() constructor. Do not schedule the returned command on its own or add it to a different composition.");
        }

        // should only be called once, so no need to precompute

        @Override
        public boolean runsWhenDisabled() {
            for (var cmd : commands) {
                if (!cmd.runsWhenDisabled()) return false;
            }
            return true;
        }

        @Override
        public InterruptionBehavior getInterruptionBehavior() {
            for (var cmd : commands) {
                if (cmd.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                    return InterruptionBehavior.kCancelSelf;
                }
            }
            return InterruptionBehavior.kCancelIncoming;
        }
    }

    /**
     * Mark one or more commands as <strong>asynchronous</strong> in the
     * arguments list of an {@link AsyncSequence}. The returned command can
     * ONLY be passed to the AsyncSequence constructor, and cannot be scheduled
     * on its own or added to any other type of command composition.
     *
     * @see AsyncSequence
     */
    public static Command async(Command... commands) {
        return new AsyncWrapper(commands);
    }

    private final boolean runsWhenDisabled;
    private final InterruptionBehavior interruptionBehavior;

    private final Command[] commands;

    private int index = -1;
    private final Set<Command> currentAsync = new LinkedHashSet<>();

    public AsyncSequence(Command... commands) {
        if (commands.length == 0) {
            throw new IllegalArgumentException("AsyncSequence must be constructed with at least one command.");
        }

        this.commands = commands;

        boolean runsWhenDisabled = true, cancelIncoming = true;
        for (var cmd : commands) {
            addRequirements(cmd.getRequirements());
            runsWhenDisabled &= cmd.runsWhenDisabled();
            cancelIncoming &= cmd.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming;
        }
        this.runsWhenDisabled = runsWhenDisabled;
        this.interruptionBehavior = cancelIncoming ? InterruptionBehavior.kCancelIncoming : InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initialize() {
        initNext();
    }

    @Override
    public void execute() {
        // tick already running async commands
        for (var iter = currentAsync.iterator(); iter.hasNext();) {
            var async = iter.next();
            async.execute();
            if (async.isFinished()) {
                async.end(false);
                iter.remove();
            }
        }

        if (index != -1) {
            // tick the current sync command
            var sync = commands[index];

            sync.execute();
            if (sync.isFinished()) {
                sync.end(false);
                initNext();
            }
        }
    }

    private void initNext() {
        while (++index < commands.length) {
            var cmd = commands[index];
            cancelConflictingAsync(cmd);

            // initialize next "streak" of contiguous async commands
            if (cmd instanceof AsyncWrapper wrapper) {
                for (var async : wrapper.commands) {
                    async.initialize();
                }
                Collections.addAll(currentAsync, wrapper.commands);
            } else {
                // hit a sync command - initialize it and stop here
                cmd.initialize();
                return;
            }
        }

        // hit the end of the command array
        index = -1;
    }

    private void cancelConflictingAsync(Command cmd) {
        var requirements = cmd.getRequirements();

        for (var iter = currentAsync.iterator(); iter.hasNext();) {
            var async = iter.next();
            if (!Collections.disjoint(requirements, async.getRequirements())) {
                async.end(true);
                iter.remove();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // end any active commands and reset
        if (interrupted) {
            if (index >= 0 && index < commands.length) {
                commands[index].end(true);
            }
            for (var async : currentAsync) {
                async.end(true);
            }
        }

        index = -1;
        currentAsync.clear();
    }

    @Override
    public boolean isFinished() {
        // index is -1 if we are not running or have finished all sync commands
        return index == -1 && currentAsync.isEmpty();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return interruptionBehavior;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhenDisabled;
    }

}
