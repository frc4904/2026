package org.usfirst.frc4904.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.usfirst.frc4904.standard.commands.AsyncSequence;

import java.util.Arrays;

public interface TrajectoryCommand {

    double getDuration();

    // length of returned array may be slightly off due to rounding errors
    Pose2d[] getTrajPreview(int steps);

    default Pose2d[] getTrajPreview() {
        return getTrajPreview(PathManager.PATHPLANNER_PREVIEW_STEPS);
    }

    Pose2d getInitialPose();

    Pose2d getEndPose();

    /// COMMAND GROUP WRAPPERS

    class ParallelPathPlannerGroup extends ParallelCommandGroup implements TrajectoryWrapperSingle {

        private final TrajectoryCommand trajCommand;

        public ParallelPathPlannerGroup(Command... commands) {
            super(commands);

            trajCommand = findTrajCommand(commands);

            if (trajCommand == null) {
                throw new IllegalArgumentException("ParallelPathPlannerGroup must be constructed with exactly one TrajectoryCommand");
            }
        }

        @Override
        public TrajectoryCommand getCommand() {
            return trajCommand;
        }

    }

    class ParallelDeadlinePathPlannerGroup extends ParallelDeadlineGroup implements TrajectoryWrapperSingle {

        private final TrajectoryCommand trajCommand;

        public ParallelDeadlinePathPlannerGroup(Command deadline, Command... otherCommands) {
            super(deadline, otherCommands);

            Command[] commands = new Command[otherCommands.length + 1];
            commands[0] = deadline;
            System.arraycopy(otherCommands, 0, commands, 1, otherCommands.length);
            trajCommand = findTrajCommand(commands);

            if (trajCommand == null) {
                throw new IllegalArgumentException("ParallelDeadlinePathPlannerGroup must be constructed with exactly one TrajectoryCommand");
            }
        }

        @Override
        public TrajectoryCommand getCommand() {
            return trajCommand;
        }

    }

    /**
     * Note: Using {@link SequentialCommandGroup#addCommands(Command...) addCommands()} will not
     * add trajectories to the preview.
     */
    class SequentialPathPlannerGroup extends SequentialCommandGroup implements TrajectoryWrapperMulti {

        private final double duration;
        private final TrajectoryCommand[] trajCommands;

        public SequentialPathPlannerGroup(Command... commands) {
            super(commands);

            trajCommands = filterTrajCommands(commands);

            if (trajCommands.length == 0) {
                throw new IllegalArgumentException("Cannot construct a SequentialPathPlannerGroup without any TrajectoryCommands");
            }

            double dur = 0;
            for (var cmd : trajCommands) {
                dur += cmd.getDuration();
            }
            duration = dur;
        }

        @Override
        public double getDuration() {
            return duration;
        }

        @Override
        public TrajectoryCommand[] getCommands() {
            return trajCommands;
        }

    }

    /**
     * Note: Path trajectory is only assembled from synchronous commands.
     */
    class AsyncPathPlannerSequence extends AsyncSequence implements TrajectoryWrapperMulti {

        private final double duration;
        private final TrajectoryCommand[] trajCommands;

        public AsyncPathPlannerSequence(Command... commands) {
            super(commands);

            trajCommands = filterTrajCommands(commands);

            if (trajCommands.length == 0) {
                throw new IllegalArgumentException("Cannot construct an AsyncPathPlannerSequence without any synchronous TrajectoryCommands");
            }

            double dur = 0;
            for (var cmd : trajCommands) {
                dur += cmd.getDuration();
            }
            duration = dur;
        }

        @Override
        public double getDuration() {
            return duration;
        }

        @Override
        public TrajectoryCommand[] getCommands() {
            return trajCommands;
        }

    }

    private static TrajectoryCommand[] filterTrajCommands(Command... commands) {
        return Arrays.stream(commands)
                     .filter(cmd -> cmd instanceof TrajectoryCommand)
                     .toArray(TrajectoryCommand[]::new);
    }

    private static TrajectoryCommand findTrajCommand(Command... commands) {
        var trajCommands = filterTrajCommands(commands);
        return trajCommands.length == 1 ? trajCommands[0] : null;
    }

}

// utility for wrapping existing sequential command groups with TrajectoryCommand
interface TrajectoryWrapperMulti extends TrajectoryCommand {

    TrajectoryCommand[] getCommands();

    @Override
    default Pose2d[] getTrajPreview(int totalSteps) {
        return Arrays.stream(getCommands())
                     .flatMap(cmd -> {
                         int steps = (int) Math.round(cmd.getDuration() / getDuration() * totalSteps);
                         return Arrays.stream(cmd.getTrajPreview(steps));
                     })
                     .toArray(Pose2d[]::new);
    }

    @Override
    default Pose2d getInitialPose() {
        return getCommands()[0].getInitialPose();
    }

    @Override
    default Pose2d getEndPose() {
        var commands = getCommands();
        return commands[commands.length - 1].getEndPose();
    }

}

// utility for wrapping existing parallel command groups with TrajectoryCommand
interface TrajectoryWrapperSingle extends TrajectoryCommand {

    TrajectoryCommand getCommand();

    @Override
    default double getDuration() {
        return getCommand().getDuration();
    }

    @Override
    default Pose2d[] getTrajPreview(int totalSteps) {
        return getCommand().getTrajPreview(totalSteps);
    }

    @Override
    default Pose2d getInitialPose() {
        return getCommand().getInitialPose();
    }

    @Override
    default Pose2d getEndPose() {
        return getCommand().getEndPose();
    }

}
