/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc4904.robot.Auton.PathPlannerCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.Dashboard;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveDriver;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.silly.Silly;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.Util;

public class Robot extends CommandRobotBase {

    private static final String[] AUTON_NAMES = {
        "STRET", "4", "7", "romtater", "aaahhh", "4904"
    };

    private static final FieldObject2d
        autonPreview = Dashboard.previewField.getObject("auton_preview"),
        autonStart = Dashboard.previewField.getRobotObject(),
        autonEnd = Dashboard.previewField.getObject("auton_end");

    private Boolean lastAutonFlip; // FIX THIS CANNOT BE RIGHT4
    // alliance is not guaranteed to be set on startup
    // so we need to check it multiple times and flip the paths if necessary.
    // the chooser will remain selecting the auton with the same name even if the value is replaced.
    private void updateAuton() {
        boolean flipAuton = DriverStation.getAlliance().orElse(null) == Alliance.Red;

        // always true first time since lastAutonFlip is null
        if (lastAutonFlip != null && flipAuton != lastAutonFlip) {
            lastAutonFlip = flipAuton;
            Auton.initPathplanner(autonChooser, flipAuton, AUTON_NAMES);
        }
    }

    @Override
    public void initialize() {
        DriverStation.silenceJoystickConnectionWarning(true); // BEGONE

        SmartDashboard.putData("scheduler", CommandScheduler.getInstance());
        SmartDashboard.putString("Elastic Working", "YES.");
        autonChooser.setDefaultOption("none", new NoOp());
        autonChooser.addOption("straight", Auton.c_jankStraight());
        autonChooser.addOption("reverse", Auton.c_jankReverse());
        updateAuton();

        driverChooser.setDefaultOption("swerve", new SwerveDriver());

        operatorChooser.setDefaultOption("default", new DefaultOperator());

        // show selected auton path in elastic dashboard
        autonChooser.onChange(_cmd -> {
            updateAuton();

            // updateAuton() could've flipped the selected command
            Command cmd = autonChooser.getSelected();

            if (cmd instanceof PathPlannerCommand pathCmd) {
                autonPreview.setPoses(pathCmd.getTrajPreview());
                autonStart.setPose(pathCmd.traj.getInitialPose());
                autonEnd.setPose(pathCmd.traj.getEndState().pose);
            } else {
                Util.clearPose(autonPreview, autonStart, autonEnd);
            }
        });

        Component.chassis.startPoseEstimator(Translation2d.kZero);

        Silly.initialize();
    }

    @Override
    public void teleopInitialize() {
        // Component.lights.flashColor(LightSubsystem.Color.ENABLED);
    }

    @Override
    public void teleopExecute() {
        if (Component.chassis.poseEstimatorEnabled()) {
            Dashboard.liveField.setRobotPose(Component.chassis.getPoseEstimate());
        } else {
            Util.clearPose(Dashboard.liveField.getRobotObject());
        }
    }

    @Override
    public void teleopCleanup() {}

    @Override
    public void autonomousInitialize() {
        updateAuton();

        // if we are using absolute pathplanner positioning (see javadoc on constant),
        // then we're probably starting in the right place, so let's zero the pose
        // estimator assuming that we are. even if not, it's still probably a better
        // estimate than (0, 0) and it'll hopefully get adjusted with vision data.
        if (
            Auton.ABSOLUTE_PATHPLANNER_POSITIONING
            && autonChooser.getSelected() instanceof PathPlannerCommand pathCmd
        ) {
            Pose2d pose = pathCmd.traj.getInitialPose();

            Component.imu.zeroYaw(pose.getRotation().getRotations());
            Component.chassis.startPoseEstimator(pose);
        }
    }

    @Override
    public void autonomousExecute() {}

    @Override
    public void autonomousCleanup() {}

    @Override
    public void disabledInitialize() {
        Component.vision.stopPositioning("Robot disabled", false);

        Component.chassis.stop();

        CommandScheduler.getInstance().cancelAll();

        // Component.lights.flashColor(LightSubsystem.Color.DISABLED);
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

        SmartDashboard.putNumber("velocidad", Component.shooterMotorLeft.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("imu temp", Component.imu.getTemperature());
        SmartDashboard.putNumber("pdh temp", Util.fahrenheit(Component.pdh.getTemperature()));
        SmartDashboard.putNumber("match time", Timer.getMatchTime());

        SmartDashboard.putNumber("climber encoder", Component.climberEncoder.get());
        SmartDashboard.putNumber("intake encoder", Component.intakeEncoder.get());
        SmartDashboard.putNumber("ideal vel", ShooterSubsystem.getShooterVelocityForDistance(2.63));
    }

    @Override
    public void simulationInitialize() {}

    @Override
    public void simulationExecute() {}
}
