/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc4904.robot.Auton.PathPlannerCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.Dashboard;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.Util;

public class Robot extends CommandRobotBase {

    private static final FieldObject2d
        autonPreview = Dashboard.previewField.getObject("auton_preview"),
        autonStart = Dashboard.previewField.getRobotObject(),
        autonEnd = Dashboard.previewField.getObject("auton_end");

    @Override
    public void initialize() {
        DriverStation.silenceJoystickConnectionWarning(true); // BEGONE

        SmartDashboard.putData("scheduler", CommandScheduler.getInstance());
        SmartDashboard.putString("Elastic Working", "YES.");
        autonChooser.setDefaultOption("none", new NoOp());
        autonChooser.addOption("straight", Auton.c_jankStraight());
        autonChooser.addOption("reverse", Auton.c_jankReverse());
        Auton.initPathplanner(autonChooser, "STRET", "4", "7", "romtater", "aaahhh", "4904");

        driverChooser.setDefaultOption("swerve", new SwerveGain());

        operatorChooser.setDefaultOption("default", new DefaultOperator());

        // show selected auton path in elastic dashboard
        autonChooser.onChange(cmd -> {
            if (cmd instanceof PathPlannerCommand pathCmd) {
                autonPreview.setPoses(pathCmd.getTrajPreview());
                autonStart.setPose(pathCmd.traj.getInitialPose());
                autonEnd.setPose(pathCmd.traj.getEndState().pose);
            } else {
                Util.clearPose(autonPreview, autonStart, autonEnd);
            }
        });

        Component.chassis.startPoseEstimator(Pose2d.kZero);
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
    public void autonomousInitialize() {}

    @Override
    public void autonomousExecute() {}

    @Override
    public void autonomousCleanup() {}

    @Override
    public void disabledInitialize() {
        Component.vision.stopPositioning("Robot disabled", false);

        Component.chassis.stop();

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
        SmartDashboard.putNumber("imu temp", Component.imu.getTemperature());
        SmartDashboard.putNumber("match time", Timer.getMatchTime());

        SmartDashboard.putNumber("climber encoder", Component.climberEncoder.get());
        SmartDashboard.putNumber("intake encoder", Component.intakeEncoder.get());
    }
}
