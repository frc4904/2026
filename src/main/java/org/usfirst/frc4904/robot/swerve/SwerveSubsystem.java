package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.vision.TagManager;
import org.usfirst.frc4904.standard.util.CmdUtil;
import org.usfirst.frc4904.standard.util.Util;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

final class SwerveConstants {
    private SwerveConstants() {}

    static final double DRIVE_GEAR_RATIO = 1.0545; // 5.625; // motor rots/wheel rots

    static final double DRIVE_RPM = 6380; // TODO get real measurement

    static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5); // meters
    static final double ROBOT_DIAGONAL = 1.15; // meters - TODO get real measurement

    static final double WHEEL_CIRC = Math.PI * WHEEL_DIAMETER; // meters

    // m/s
    static final double LIN_SPEED = DRIVE_RPM / 60.0 / DRIVE_GEAR_RATIO * WHEEL_CIRC;
    // turns/s
    static final double ROT_SPEED = LIN_SPEED / (Math.PI * ROBOT_DIAGONAL);
}

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule[] modules;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;

    private boolean estimatorEnabled = false;

    public SwerveSubsystem(SwerveModule... modules) {
        this.modules = modules;

        kinematics = new SwerveDriveKinematics(
            Arrays.stream(modules)
                  .map(module -> module.pos)
                  .toArray(Translation2d[]::new)
        );
        estimator = new SwerveDrivePoseEstimator(
            kinematics,
            getAbsoluteRotation(),
            getModulePositions(),
            Pose2d.kZero // unused
        );

        SmartDashboard.putData("swerve/goal", this);
    }

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getModulePosition)
                     .toArray(SwerveModulePosition[]::new);
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getModuleState)
                     .toArray(SwerveModuleState[]::new);
    }

    public void startPoseEstimator(Translation2d currentPos) {
        startPoseEstimator(new Pose2d(currentPos, getAbsoluteRotation()));
    }

    public void startPoseEstimator(Pose2d currentPose) {
        estimator.resetPose(currentPose);
        estimatorEnabled = true;
    }

    public void stopPoseEstimator() {
        estimatorEnabled = false;
    }

    public boolean poseEstimatorEnabled() {
        return estimatorEnabled;
    }

    /**
     * @return The current pose estimate, based on swerve odometry and {@code addVisionPoseEstimate()} calls
     */
    public Pose2d getPoseEstimate() {
        if (!estimatorEnabled) {
            System.err.println("SwerveSubsystem.getPoseEstimate() called while pose estimator is disabled.");
            return Pose2d.kZero;
        }

        return estimator.getEstimatedPosition();
    }

    /**
     * @return The current position estimate, equivalent to {@code getPoseEstimate().getTranslation()}
     */
    public Translation2d getPositionEstimate() {
        return getPoseEstimate().getTranslation();
    }

    /**
     * @return current robot-relative {@link ChassisSpeeds} (velocity and direction) according to encoders
     */
    public ChassisSpeeds getChassisSpeeds() {
        // TODO suboptimal that we are potentially recomputing this multiple times per loop
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * @return current robot-relative velocity according to encoders
     */
    public Translation2d getVelocity() {
        ChassisSpeeds speed = getChassisSpeeds();
        return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    }

    /**
     * Drive according to joystick inputs. {@code hypot(x, y)} should not exceed 1.
     * @param translation X/Y movement from [-1, 1], wpilib coordinate system (forward, left)
     * @param theta Turn speed from [-1, 1], positive = counterclockwise
     */
    public void input(Translation2d translation, double theta) {
        Translation2d scaled = translation.times(SwerveConstants.LIN_SPEED);
        driveFieldRelative(scaled, theta * SwerveConstants.ROT_SPEED);
    }

    public Translation2d toRobotRelative(Translation2d translation) {
        return translation.rotateBy(Rotation2d.fromRotations(-getHeading()));
    }

    /**
     * Drive relative to the field.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second - not field-relative,
     *              as it represents the turning speed, not an absolute angle.
     *              Will be overridden if a c_rotateTo() command is active
     */
    public void driveFieldRelative(Translation2d translation, double theta) {
        driveRobotRelative(toRobotRelative(translation), theta);
    }

    /**
     * Drive relative to the current angle of the robot.
     * @param speeds Target velocity and rotation, deconstructed into a translation and rotation.
     *               See {@link #driveRobotRelative(Translation2d, double)}
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        driveRobotRelative(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            Units.radiansToRotations(speeds.omegaRadiansPerSecond)
        );
    }

    private Translation2d driveTranslation = Translation2d.kZero;
    private double driveTheta = 0;

    /**
     * Drive relative to the current angle of the robot.
     * @param translation Movement speed in meters per second
     *                    Will be overridden if a {@link #c_gotoPos(Supplier) c_gotoPos()} command is active
     * @param theta Rotation speed in rotations per second.
     *              Will be overridden if a {@link #c_rotateTo(double, boolean) c_rotateTo()} command is active
     */
    public void driveRobotRelative(Translation2d translation, double theta) {
        driveTranslation = translation;
        driveTheta = theta;
    }

    /** See {@link #driveRobotRelative(Translation2d, double)} */
    public void driveRobotRelative(double x, double y, double theta) {
        driveRobotRelative(new Translation2d(x, y), theta);
    }

    public void stop() {
        driveRobotRelative(0, 0, 0);
    }

    private final double[] prevHeadings = new double[50];
    private int prevHeadingIndex = 0;

    private double lastTagUpdateTime;

    private static final Translation2d CAMERA_OFFSET = new Translation2d(Units.inchesToMeters(12.5), 0);

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            var translation = posCommand != null ? posPIDEffort : driveTranslation;
            var theta = rotCommand != null ? rotPIDEffort : driveTheta;

            if (autoBrickWhenStill && Math.abs(theta) < 0.01 && translation.getNorm() < 0.01) {
                brickMode();
            } else {
                Translation2d[] translations = new Translation2d[modules.length];
                double maxMag = SwerveConstants.LIN_SPEED;

                for (int i = 0; i < modules.length; i++) {
                    Translation2d rotation = modules[i].rotToTranslation(theta);
                    Translation2d sum = translation.plus(rotation);

                    translations[i] = sum;
                    maxMag = Math.max(sum.getNorm(), maxMag);
                }

                double norm = maxMag / SwerveConstants.LIN_SPEED;

                for (int i = 0; i < modules.length; i++) {
                    Translation2d normalized = translations[i].div(norm);

                    double magnitude = normalized.getNorm();
                    modules[i].moveTo(
                        magnitude,
                        magnitude > 0 ? normalized.getAngle().getRotations() : 0
                    );
                }
            }

            for (var module : modules) module.periodic();
        }

        prevHeadings[prevHeadingIndex++] = getAbsoluteHeading();
        prevHeadingIndex %= prevHeadings.length;

        if (estimatorEnabled) {
            estimator.update(getAbsoluteRotation(), getModulePositions());

            var tags = TagManager.getTagsSince(lastTagUpdateTime);
            lastTagUpdateTime = TagManager.getLastTime();

            double now = Timer.getFPGATimestamp();

            for (var tag : tags) {
                double secondsAgo = now - tag.time();
                if (secondsAgo >= 1) continue;

                double exactIndex = prevHeadingIndex - secondsAgo / Robot.defaultPeriodSecs;
                while (exactIndex < 0) exactIndex += prevHeadings.length;
                int floorIndex = (int) exactIndex;
                double heading;
                if (exactIndex == floorIndex) {
                    heading = prevHeadings[floorIndex];
                } else {
                    heading = Util.lerp(
                        exactIndex % 1,
                        prevHeadings[floorIndex],
                        prevHeadings[(floorIndex + 1) % prevHeadings.length]
                    );
                }

                Translation2d robotToTagRR = tag.pos().getTranslation().toTranslation2d();
                Translation2d robotToTagFR = robotToTagRR.rotateBy(Rotation2d.fromRotations(heading));

                Translation2d tagFR = tag.fieldPos().getTranslation().toTranslation2d();
                Translation2d robotFR = tagFR.minus(robotToTagFR);

                double stdDev = Math.pow(robotToTagRR.getNorm() / 2, 2);
                estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, 1e100));

                estimator.addVisionMeasurement(
                    new Pose2d(robotFR, getAbsoluteRotation()),
                    now // TODO VISION use frame time (probably fixed now?)
                );
            }

        }
    }

    double getHeading() {
        return Component.imu.getYaw();
    }

    /**
     * Heading that accounts for alliance flip.
     * <p>
     * Bottom right corner of the blue side is (0, 0, 0).
     * Accordingly, blue —> red is 0deg, red —> blue is 180deg.
     */
    double getAbsoluteHeading() {
        double heading = getHeading();
        return Robot.isRedAlliance() ? (heading + 0.5) % 1 : heading;
    }

    Rotation2d getRotation() {
        return Component.imu.getRotation2d();
    }

    Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromRotations(getAbsoluteHeading());
    }

    /// COMMANDS

    private RotateCommand rotCommand; // non-null when a rot command is running
    private double rotPIDEffort;

    /**
     * @param theta Field-relative angle to rotate to
     * @param useAbsoluteHeading When {@code true}, flips the IMU angle when on red alliance. This makes
     *                       the angles "properly" field-relative instead of "alliance-relative"
     * @return A command that uses PID to rotate to the provided angle.
     *         Overrides any rotation from any other drive commands or methods while the command is running
     */
    public Command c_rotateTo(double theta, boolean useAbsoluteHeading) {
        return c_rotateTo(() -> theta, useAbsoluteHeading);
    }

    /**
     * @param getTheta Supplier of field-relative angles to rotate to
     * @param useAbsoluteHeading When {@code true}, flips the IMU angle when on red alliance. This makes
     *                       the angles "properly" field-relative instead of "alliance-relative"
     * @return A command that uses PID to rotate to the provided angle.
     *         Overrides any rotation from any other drive commands or methods while the command is running
     */
    public Command c_rotateTo(Supplier<Double> getTheta, boolean useAbsoluteHeading) {
        return new RotateCommand(getTheta, useAbsoluteHeading);
    }

    /**
     * @param getTheta Supplies doubles representing the difference between the current heading and target angle
     * @return A command that uses PID to rotate to the provided angle.
     *         Overrides any rotation from any other drive commands or methods while the command is running
     */
    public Command c_controlRotation(DoubleSupplier getTheta) {
        return new RotateCommand(() -> getTheta.getAsDouble() + getHeading(), false);
    }

    private class RotateCommand extends Command {

        private static final double ROT_DIST_THRESHOLD = Units.degreesToRotations(5);

        private final PIDController rotPID;
        private final Supplier<Double> getTheta;
        private final boolean useAbsoluteHeading;

        RotateCommand(Supplier<Double> getTheta, boolean useAbsoluteHeading) {
            this.getTheta = getTheta;
            this.useAbsoluteHeading = useAbsoluteHeading;

            rotPID = new PIDController(100, 10, 0);
            rotPID.enableContinuousInput(0, 1);
            // don't require swerve subsystem so that it can run in parallel to other swerve commands
        }

        Double lastGoal;
        boolean done;

        @Override
        public void initialize() {
            // manually cancel any other active rotate command
            if (rotCommand != null) rotCommand.cancel();
            rotCommand = this;

            rotPID.reset();
            done = false;
        }

        @Override
        public void execute() {
            double current = useAbsoluteHeading ? getAbsoluteHeading() : getHeading();
            Double goal = getTheta.get();

            if (goal == null) {
                if (lastGoal == null) {
                    rotPIDEffort = 0;
                    return;
                } else {
                    goal = lastGoal;

                    double diff = Math.abs(MathUtil.inputModulus(current - goal, -0.5, 0.5));
                    if (diff <= ROT_DIST_THRESHOLD) {
                        done = true;
                    }
                }
            } else {
                lastGoal = goal;
            }

            rotPIDEffort = Util.clamp(
                rotPID.calculate(current, goal),
                -SwerveConstants.ROT_SPEED,
                SwerveConstants.ROT_SPEED
            );
        }

        @Override
        public void end(boolean interrupted) {
            rotCommand = null;
        }

        @Override
        public boolean isFinished() {
            return done;
        }
    }

    private PositionCommand posCommand; // non-null when a pos command is running
    private Translation2d posPIDEffort = Translation2d.kZero;

    /**
     * @param getPos Supplier of field-relative translations to go to.
     *               Return {@code null} to signal that the last pose was the final pose of the path, and the command
     *               will end once the robot is within {@code PositionCommand.DISTANCE_THRESHOLD} of the final pose.
     * @return A command that uses PID and the {@link #startPoseEstimator(Pose2d) pose estimator} to follow a stream of positions.
     *         Overrides any movement from any other drive commands or methods while the command is running
     */
    public Command c_gotoPos(Supplier<Translation2d> getPos) {
        return new PositionCommand(getPos);
    }

    private class PositionCommand extends Command {

        private static final double DISTANCE_THRESHOLD = 0.1;

        private final PIDController posPID;
        private final Supplier<Translation2d> getPos;

        PositionCommand(Supplier<Translation2d> getPos) {
            this.getPos = getPos;

            posPID = new PIDController(100, 15, 0);
            // don't require swerve subsystem so that it can run in parallel to other swerve commands
        }

        Translation2d lastGoal;
        boolean done;

        @Override
        public void initialize() {
            // manually cancel any other active position command
            if (posCommand != null) posCommand.cancel();
            posCommand = this;

            posPID.reset();
            done = false;
        }

        @Override
        public void execute() {
            Translation2d current = getPositionEstimate();
            Translation2d goal = getPos.get();

            if (goal == null) {
                if (lastGoal == null) {
                    posPIDEffort = Translation2d.kZero;
                    return;
                } else {
                    goal = lastGoal;
                    done |= current.getDistance(goal) <= DISTANCE_THRESHOLD;
                }
            } else {
                lastGoal = goal;
            }

            Translation2d diff = goal.minus(current);

            double pidEffort = Util.clamp(
                posPID.calculate(0, diff.getNorm()),
                -SwerveConstants.LIN_SPEED,
                SwerveConstants.LIN_SPEED
            );

            //                  pid              convert to robot relative
            posPIDEffort = diff.times(pidEffort).rotateBy(getAbsoluteRotation().unaryMinus());
        }

        @Override
        public void end(boolean interrupted) {
            posCommand = null;
        }

        @Override
        public boolean isFinished() {
            return done;
        }
    }

    /**
     * Combination of {@link #c_rotateTo(Supplier, boolean) c_rotateTo} and {@link #c_gotoPos(Supplier) c_gotoPos}.
     */
    public Command c_gotoPose(Supplier<? extends Pose2d> getPose, boolean useAbsoluteHeading) {
        return CmdUtil.withState(
            getPose,
            state -> new ParallelCommandGroup(
                c_rotateTo(() -> state.get() != null ? state.get().getRotation().getRotations() : null, useAbsoluteHeading),
                c_gotoPos(() -> state.get() != null ? state.get().getTranslation() : null)
            ),
            this
        );
    }

    /**
     * Stop the wheels.
     */
    public Command c_stop() {
        return runOnce(this::stop);
    }

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(Translation2d translation, double theta) {
        return run(() -> driveFieldRelative(translation, theta)).finallyDo(this::stop);
    }

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(double x, double y, double theta) {
        Translation2d translation = new Translation2d(x, y);
        return run(() -> driveFieldRelative(translation, theta)).finallyDo(this::stop);
    }

    /**
     * Hold a robot-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveRobotRelative(Translation2d, double)}
     */
    public Command c_driveRobotRelative(Translation2d translation, double theta) {
        return run(() -> driveRobotRelative(translation, theta)).finallyDo(this::stop);
    }

    /**
     * Hold a robot-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveRobotRelative(Translation2d, double)}
     */
    public Command c_driveRobotRelative(double x, double y, double theta) {
        Translation2d translation = new Translation2d(x, y);
        return run(() -> driveRobotRelative(translation, theta)).finallyDo(this::stop);
    }

    /**
     * Drive according to inputs provided by the suppliers.
     * <p>
     * See {@link #input(Translation2d, double)}
     */
    public Command c_input(Supplier<? extends Translation2d> translation, DoubleSupplier theta) {
        return run(() -> input(translation.get(), theta.getAsDouble())).finallyDo(this::stop);
    }

    /// MISC CONFIG

    public void resetOdometry() {
        Component.imu.zeroYaw();
        estimator.resetRotation(getAbsoluteRotation());
    }

    /**
     * Zero the rotation encoders for all swerve modules.
     */
    public void zero() {
        System.out.println("zeroed");
        for (var module : modules) module.zero();
    }

    /**
     * Flip current zero position by 180deg.
     */
    public void flipZero() {
        System.out.println("flipped zero");
        for (var module : modules) module.flipZero();
    }

    public void setMotorBrake(boolean brake) {
        for (var module : modules) module.setMotorBrake(brake);
    }

    public void brickMode() {
        for (var module : modules) module.brickMode();
    }

    private boolean autoBrickWhenStill = false;

    public void setAutoBrickWhenStill(boolean enabled) {
        autoBrickWhenStill = enabled;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Robot Angle", this::getHeading, null);

        for (var module : modules) module.addSendableProps(builder);
    }
}
