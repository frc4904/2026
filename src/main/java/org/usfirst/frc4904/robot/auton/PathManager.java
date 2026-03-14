package org.usfirst.frc4904.robot.auton;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.json.simple.parser.ParseException;
import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.Dashboard;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.silly.console;
import org.usfirst.frc4904.standard.util.Util;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;

public final class PathManager {
    private PathManager() {}

    /**
     * When {@code false}, PathPlanner paths will be moved so that the starting position
     * of the path lines up with the robot position when the auton starts.
     * <p>
     * When {@code true}, paths will not be moved, and the robot will try to get to the
     * absolute position of the path on the field.
     */
    public static final boolean ABSOLUTE_PATHPLANNER_POSITIONING = true;

    public static boolean shouldFlip() {
        return Robot.isRedAlliance();
    }

    // apparently cannot be higher than 85 (????) - see javadoc for FieldObject2d.setPoses()
    private static final int PATHPLANNER_PREVIEW_STEPS = 50;

    private static final double PATHPLANNER_SLOWDOWN_FACTOR = 4;

    private static final FieldObject2d liveTraj = Dashboard.liveField.getObject("auton_traj");
    private static final FieldObject2d liveTarget = Dashboard.liveField.getObject("auton_next");

    private static final Map<String, Command> pathCache = new HashMap<>();

    static RobotConfig pathPlannerConfig;
    static {
        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner config:\n" + e.getMessage());
        }
    }

    public static Command c_path(String name) {
        // return pathCache.computeIfAbsent(name, PathManager::loadPathCommand);
        return loadPathCommand(name);
    }

    private static Command loadPathCommand(String file) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(file);
            PathPlannerTrajectory traj = path.getIdealTrajectory(pathPlannerConfig).orElseThrow();

            console.log("AUTON HALP - path loaded", file);

            return new PathPlannerCommand(traj);
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "':\n" + e.getMessage());
        } catch (NoSuchElementException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "'. Paths must have an ideal starting state.");
        }

        console.log("AUTON HALP - FAILED TO LOAD PATH COMMAND!!!!!!");
        return new NoOp();
    }

    private static Pose2d[] makeTrajPreview(PathPlannerTrajectory traj, int steps) {
        return makeTrajPreview(traj, traj.getInitialPose(), steps);
    }

    private static Pose2d[] makeTrajPreview(PathPlannerTrajectory traj, Pose2d start, int steps) {
        double dur = traj.getTotalTimeSeconds();
        double timePerStep = dur / steps;

        Pose2d[] poses = new Pose2d[steps];
        for (int step = 0; step < steps; step++) {
            poses[step] = sampleTraj(traj, start, step * timePerStep);
        }
        return poses;
    }

    private static Pose2d sampleTraj(PathPlannerTrajectory traj, Pose2d start, double time) {
        Pose2d pose = traj.sample(time).pose, initial = traj.getInitialPose();
        if (start == initial) return pose; // should be exactly equal if getInitialPose() was used
        return start.plus(new Transform2d(initial, pose));
    }

    public static class PathPlannerCommand extends Command implements TrajectoryCommand {

        public PathPlannerTrajectory traj;
        private final double duration;

        @Override
        public double getDuration() {
            return duration;
        }

        @Override
        public Pose2d[] getTrajPreview(int steps) {
            updateFlip();
            return makeTrajPreview(traj, steps);
        }

        @Override
        public Pose2d getInitialPose() {
            updateFlip();
            return traj.getInitialPose();
        }

        @Override
        public Pose2d getEndPose() {
            updateFlip();
            return traj.getEndState().pose;
        }

        private double startTime;
        private Pose2d start;
        private boolean atEnd;

        private final Command gotoPoseCommand;

        private PathPlannerCommand(PathPlannerTrajectory traj) {
            this.traj = traj;
            updateFlip();

            duration = traj.getTotalTimeSeconds();

            // effectively a WrapperCommand of this c_gotoPose() command
            // ...BUT WrapperCommand requires that the command is provided in the super() call,
            // which can't reference class fields like `startTime` or `offset`.
            // so i'll wrap it myself i guess.
            gotoPoseCommand = Component.chassis.c_gotoPose(() -> {
                if (atEnd) return null;

                double time = (Timer.getFPGATimestamp() - startTime) / PATHPLANNER_SLOWDOWN_FACTOR;
                if (time >= duration) atEnd = true;

                Pose2d target = sampleTraj(traj, start, time);
                liveTarget.setPose(target);
                return target;
            }, true);
        }

        private boolean lastFlip;
        private void updateFlip() {
            if (lastFlip != shouldFlip()) {
                lastFlip = !lastFlip;
                traj = traj.flip();
            }
        }

        @Override
        public void initialize() {
            console.log("AUTON HALP - PathPlannerCommand.initialize()");

            updateFlip();

            atEnd = false;
            startTime = Timer.getFPGATimestamp();
            Pose2d current = Component.chassis.getPoseEstimate();

            start = ABSOLUTE_PATHPLANNER_POSITIONING
                ? traj.getInitialPose()
                // orientation/rotation of path is always field relative
                : new Pose2d(current.getTranslation(), Rotation2d.kZero);

            int steps = Math.min(PATHPLANNER_PREVIEW_STEPS, (int) Math.round(duration * 10));
            liveTraj.setPoses(makeTrajPreview(traj, start, steps));

            gotoPoseCommand.initialize();
        }

        @Override
        public void execute() {
            console.log("AUTON HALP - PathPlannerCommand.execute()");

            gotoPoseCommand.execute();
        }

        @Override
        public void end(boolean interrupted) {
            console.log("AUTON HALP - PathPlannerCommand.end()", interrupted);

            Util.clearPose(liveTraj, liveTarget);

            gotoPoseCommand.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return gotoPoseCommand.isFinished();
        }
    }

    /**
     * Note: Using {@link SequentialCommandGroup#addCommands(Command...) addCommands()} will not
     * add trajectories to the preview.
     */
    public static class SequentialPathPlannerGroup extends SequentialCommandGroup implements TrajectoryCommand {

        private final double duration;
        private final TrajectoryCommand[] trajCommands;

        public SequentialPathPlannerGroup(Command... commands) {
            super(commands);

            trajCommands = Arrays.stream(commands)
                                 .filter(cmd -> cmd instanceof TrajectoryCommand)
                                 .toArray(TrajectoryCommand[]::new);

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
        public Pose2d[] getTrajPreview(int totalSteps) {
            return Arrays.stream(trajCommands)
                         .flatMap(cmd -> {
                             int steps = (int) Math.round(cmd.getDuration() / duration * totalSteps);
                             return Arrays.stream(cmd.getTrajPreview(steps));
                         })
                         .toArray(Pose2d[]::new);
        }

        @Override
        public Pose2d getInitialPose() {
            return trajCommands[0].getInitialPose();
        }

        @Override
        public Pose2d getEndPose() {
            return trajCommands[trajCommands.length - 1].getEndPose();
        }

    }

    public interface TrajectoryCommand {

        double getDuration();

        // length of returned array may be slightly off due to rounding errors
        Pose2d[] getTrajPreview(int steps);

        default Pose2d[] getTrajPreview() {
            return getTrajPreview(PATHPLANNER_PREVIEW_STEPS);
        }

        Pose2d getInitialPose();

        Pose2d getEndPose();

    }

}
