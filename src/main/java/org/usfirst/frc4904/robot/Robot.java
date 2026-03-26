/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.Dashboard;
import org.usfirst.frc4904.robot.auton.Auton;
import org.usfirst.frc4904.robot.auton.PathManager;
import org.usfirst.frc4904.robot.auton.TrajectoryCommand;
import org.usfirst.frc4904.robot.humaninterface.drivers.RuffyDriver;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveDriver;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.subsystems.LightSubsystem;
import org.usfirst.frc4904.robot.vision.TagManager;
import org.usfirst.frc4904.robot.vision.TagManager.Tag;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.silly.Silly;
import org.usfirst.frc4904.standard.util.Storage;
import org.usfirst.frc4904.standard.util.Util;

import java.util.List;
import java.util.prefs.Preferences;

import static org.usfirst.frc4904.robot.RobotMap.USE_RUFFY_DRIVER;

public class Robot extends CommandRobotBase {

    private final boolean ADVANTAGEKIT_LOGS = true;

    private static final FieldObject2d
        autonPreview = Dashboard.previewField.getObject("auton_preview"),
        autonStart = Dashboard.previewField.getRobotObject(),
        autonEnd = Dashboard.previewField.getObject("auton_end");

    @Override
    public void initialize() {
        DriverStation.silenceJoystickConnectionWarning(true); // BEGONE

        SmartDashboard.putData("scheduler", CommandScheduler.getInstance()); // TODO: make Logger
        Logger.recordOutput("Elastic Working", "YES.");

        // basic autons
        autonChooser.setDefaultOption("none", new NoOp());
        autonChooser.addOption("straight", Auton.c_jankStraight());
        autonChooser.addOption("reverse", Auton.c_jankReverse());

        // pathplanner sequences
        autonChooser.addOption("shoot left", Auton.c_shootLeft());
        autonChooser.addOption("shoot straight", Auton.c_shootStraight());
        autonChooser.addOption("shoot center left", Auton.c_shootCenterLeft());
        autonChooser.addOption("shoot right", Auton.c_shootRight());
        autonChooser.addOption("shoot center right", Auton.c_shootCenterRight());
        autonChooser.addOption("shoot n' climb left", Auton.c_shootAndClimbLeft());
        autonChooser.addOption("shoot n' climb center left", Auton.c_shootAndClimbCenterLeft());
        autonChooser.addOption("direct climb center left", Auton.c_climbCenterLeft());
        autonChooser.addOption("shoot n' climb left from hell", Auton.c_shootAndClimbFromHell());

        // pathplanner paths
        String[] names = { "STRET", "romtater", "aaahhh", "go", "climbnew" };
        for (var name : names) {
            autonChooser.addOption("(path) " + name, PathManager.c_path(name));
        }

        // drivers
        driverChooser.setDefaultOption("swerve", USE_RUFFY_DRIVER ? new RuffyDriver() : new SwerveDriver());

        // operators
        operatorChooser.setDefaultOption("default", new DefaultOperator());

        // show selected auton path in elastic dashboard
        autonChooser.onChange(cmd -> {
            if (cmd instanceof TrajectoryCommand pathCmd) {
                autonPreview.setPoses(pathCmd.getTrajPreview());
                autonStart.setPose(pathCmd.getInitialPose());
                autonEnd.setPose(pathCmd.getEndPose());
            } else {
                Util.clearPose(autonPreview, autonStart, autonEnd);
            }
        });

        Component.chassis.startPoseEstimator(Translation2d.kZero);

        Silly.initialize();

        double test = Storage.getDouble("test", 0);
        SmartDashboard.putNumber("le test", test);
        Storage.setDouble("test", test + 1);
    }

    @Override
    public void teleopInitialize() {
        Component.chassis.stop();

        Component.lights.flashColor(LightSubsystem.Color.TELEOP);
    }

    @Override
    public void teleopExecute() {}

    @Override
    public void teleopCleanup() {}

    @Override
    public void autonomousInitialize() {
        Component.lights.flashColor(LightSubsystem.Color.AUTON);

        // if we are using absolute pathplanner positioning (see javadoc on constant),
        // then we're probably starting in the right place, so let's zero the pose
        // estimator assuming that we are. even if not, it's still probably a better
        // estimate than (0, 0) and it'll hopefully get adjusted with vision data.
        if (
            PathManager.ABSOLUTE_PATHPLANNER_POSITIONING
            && autonChooser.getSelected() instanceof TrajectoryCommand pathCmd
        ) {
            Pose2d pose = pathCmd.getInitialPose();

            Component.imu.zeroYaw(pose.getRotation().getRotations() + (Robot.isRedAlliance() ? 0.5 : 0));
            Component.chassis.startPoseEstimator(pose);
        }
    }

    @Override
    public void autonomousExecute() {}

    @Override
    public void autonomousCleanup() {}

    @Override
    public void disabledInitialize() {
        Component.chassis.stop();

        CommandScheduler.getInstance().cancelAll();

        Component.lights.flashColor(LightSubsystem.Color.DISABLED);
    }

    @Override
    public void disabledExecute() {}

    @Override
    public void disabledCleanup() {
        Component.chassis.setMotorBrake(true);
    }

    @Override
    public void testInitialize() {}

    @Override
    public void testExecute() {}

    @Override
    public void testCleanup() {}

    @Override
    public void alwaysExecute() {
        Silly.periodic();

        if (Component.chassis.poseEstimatorEnabled()) {
            Dashboard.liveField.setRobotPose(Component.chassis.getPoseEstimate());
        } else {
            Util.clearPose(Dashboard.liveField.getRobotObject());
        }

        // AdvantageKit Logs

        if (ADVANTAGEKIT_LOGS) {
            // Swerve
            if (Component.chassis.poseEstimatorEnabled()) {
                Logger.recordOutput("Swerve/PoseEstimate", Component.chassis.getPoseEstimate());
            }
            Logger.recordOutput("Swerve/ChassisSpeeds", Component.chassis.getChassisSpeeds());

            // Vision

            List<Tag> tags = TagManager.getTags();
            int[] tagIds = tags.stream().mapToInt(Tag::id).toArray();
            Pose3d[] poses = tags.stream().map(Tag::fieldPos).toArray(Pose3d[]::new);
            Logger.recordOutput("Vision/Tags", tagIds);
            Logger.recordOutput("Vision/TagPoses", poses);

            // Mechanisms

            Logger.recordOutput("Climber/Height", Component.climber.getHeight());
            Logger.recordOutput("Intake/Angle", Component.intake.getAngle());
            
            Logger.recordOutput("Shooter/Current", Component.shooterMotorLeft.getSupplyCurrent().getValueAsDouble());
        }

        Logger.recordOutput("Swerve/ChassisSpeeds", Component.chassis.getChassisSpeeds());
        Logger.recordOutput("Swerve/ModuleStates", Component.chassis.getModuleStates());

        // Vision

        List<Tag> tags = TagManager.getTags();
        int[] tagIds = tags.stream().mapToInt(Tag::id).toArray();
        Pose3d[] poses = tags.stream().map(Tag::fieldPos).toArray(Pose3d[]::new);
        Logger.recordOutput("Vision/Tags", tagIds);
        Logger.recordOutput("Vision/TagPoses", poses);

        // Mechanisms

        Logger.recordOutput("Climber/Encoder", Component.climberEncoder.get());
        Logger.recordOutput("Intake/Encoder", Component.intakeEncoder.getAbsolute());
        Logger.recordOutput("Shooter/Velocity", Component.shooterMotorLeft.getVelocity().getValueAsDouble());

        // Misc

        Logger.recordOutput("IMU/Temp", Component.imu.getTemperature());
        Logger.recordOutput("IMU/Yaw", Component.imu.getYaw());
        Logger.recordOutput("PDH/Temp", Util.fahrenheit(Component.pdh.getTemperature()));
        Logger.recordOutput("Game/MatchTime", Timer.getMatchTime());
    }

    @Override
    public void simulationInitialize() {}

    @Override
    public void simulationExecute() {}
}
