package org.usfirst.frc4904.robot.auton;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.json.simple.parser.ParseException;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.Dashboard;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.Util;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;

public final class PathManager {

    /**
     * When {@code false}, PathPlanner paths will be moved so that the starting position
     * of the path lines up with the robot position when the auton starts.
     * <p>
     * When {@code true}, paths will not be moved, and the robot will try to get to the
     * absolute position of the path on the field.
     */
    public static final boolean ABSOLUTE_PATHPLANNER_POSITIONING = false;

    // apparently cannot be higher than 85 (????) - see javadoc for FieldObject2d.setPoses()
    private static final int PATHPLANNER_PREVIEW_STEPS = 50;

    private static final double PATHPLANNER_SLOWDOWN_FACTOR = 1.5;

    private static final FieldObject2d liveTraj = Dashboard.liveField.getObject("auton_traj");
    private static final FieldObject2d liveTarget = Dashboard.liveField.getObject("auton_next");

    private static final Map<String, PathPlannerCommand> paths = new HashMap<>();

    static RobotConfig pathPlannerConfig;
    static {
        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner config:\n" + e.getMessage());
        }
    }

    private PathManager() {}

    // can be called multiple times with different values of 'flip' and paths will be overridden
    public static void init(SendableChooser<PathPlannerCommand> chooser, boolean flip, String... names) {
        if (pathPlannerConfig == null) return;

        paths.clear();
        for (var name : names) {
            Command cmd = c_pathPlanner(name, flip);

            if (cmd instanceof PathPlannerCommand pathCmd) {
                paths.put(name, pathCmd);
                chooser.addOption(name, pathCmd);
            }
        }
    }

    public static Command c_path(String name) {
        if (!paths.containsKey(name)) {
            System.err.println("No loaded PathPlanner path '" + name + "'");
            return new NoOp();
        }

        return paths.get(name);
    }

    public static Command c_pathPlanner(String file, boolean flip) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(file);
            PathPlannerTrajectory traj = path.getIdealTrajectory(pathPlannerConfig).orElseThrow();

            return new PathPlannerCommand(flip ? traj.flip() : traj);
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "':\n" + e.getMessage());
        } catch (NoSuchElementException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "'. Paths must have an ideal starting state.");
        }

        return new NoOp();
    }

    public static class PathPlannerCommand extends Command {

        public final PathPlannerTrajectory traj;
        public final double duration;

        private Pose2d[] trajPreview; // cache
        public Pose2d[] getTrajPreview() {
            if (trajPreview != null) return trajPreview;
            return trajPreview = makeTrajPreview(traj);
        }


        private double startTime;
        private Pose2d start;
        private boolean atEnd;

        private final Command gotoPoseCommand;

        private PathPlannerCommand(PathPlannerTrajectory traj) {
            this.traj = traj;
            duration = traj.getTotalTimeSeconds();

            addRequirements(Component.chassis);

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

        @Override
        public void initialize() {
            atEnd = false;
            startTime = Timer.getFPGATimestamp();
            Pose2d current = Component.chassis.getPoseEstimate();

            start = ABSOLUTE_PATHPLANNER_POSITIONING
                ? traj.getInitialPose()
                // orientation/rotation of path is always field relative
                : new Pose2d(current.getTranslation(), Rotation2d.kZero);

            liveTraj.setPoses(makeTrajPreview(traj, start));

            gotoPoseCommand.initialize();
        }

        @Override
        public void execute() {
            gotoPoseCommand.execute();
        }

        @Override
        public void end(boolean interrupted) {
            Util.clearPose(liveTraj, liveTarget);

            gotoPoseCommand.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return gotoPoseCommand.isFinished();
        }
    }

    private static Pose2d[] makeTrajPreview(PathPlannerTrajectory traj) {
        return makeTrajPreview(traj, traj.getInitialPose());
    }

    private static Pose2d[] makeTrajPreview(PathPlannerTrajectory traj, Pose2d start) {
        double dur = traj.getTotalTimeSeconds();
        int steps = Math.min(PATHPLANNER_PREVIEW_STEPS, (int) Math.round(dur * 10));
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

}
