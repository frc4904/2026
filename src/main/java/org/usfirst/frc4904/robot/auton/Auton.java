package org.usfirst.frc4904.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.auton.TrajectoryCommand.AsyncPathPlannerSequence;
import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.standard.humaninput.Operator;

import static org.usfirst.frc4904.standard.commands.AsyncSequence.async;

public final class Auton {
    private Auton() {}

    /// JANK

    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(3, 0, 0).withTimeout(2)
        );
    }

    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(-3, 0, 0).withTimeout(2)
        );
    }

    /// PATHPLANNER

    private static Command c_shoot(String type, boolean extraTime) {
        return new AsyncPathPlannerSequence(
            async(Component.intake.c_retract()),
            PathManager.c_path("shoot " + type),
            // after some time, put the intake up
            // async(CmdUtil.delayed(extraTime ? 6 : 4, Component.intake.c_retract())),
            Component.shooter.c_smartShoot().withTimeout(Operator.SHOOT_INDEXER_DELAY),
            new ParallelCommandGroup(
                Component.shooter.c_smartShoot(),
                ShooterSubsystem.c_smartShootAlign(),
                Component.indexer.c_forward(true),
                Component.intake.c_intake(),
                Component.intake.c_wobble()
            ).withTimeout(extraTime ? 15 : 6)
        );
    }

    private static Command c_shootAndClimb(String type) {
        return new AsyncPathPlannerSequence(
            async(Component.climber.c_gotoUp()),
            c_shoot(type, false),
            PathManager.c_path("climb from shoot " + type),
            Component.climber.c_gotoDown()
        );
    }

    public static Command c_climbFromHell() {
        return new AsyncPathPlannerSequence(
            async(Component.climber.c_gotoUp()),
            PathManager.c_path("climb from hell"),
            new ParallelCommandGroup(
                Component.shooter.c_smartShoot(),
                ShooterSubsystem.c_smartShootAlign(),
                Component.indexer.c_forward(true),
                Component.intake.c_intake(),
                Component.intake.c_retract(),
                Component.intake.c_wobble()
            ).withTimeout(6),
            c_sams5StepPlanToWealthAndHappiness(),
            Component.climber.c_gotoDown()
        );
    }

    public static Command c_ragebait() {
        return new AsyncPathPlannerSequence(
            async(Component.climber.c_gotoUp()),
            PathManager.c_path("climb from hell"),
            c_sams5StepPlanToWealthAndHappiness(),
            Component.climber.c_gotoDown().withTimeout(2.5),
            Component.climber.c_gotoUp().withTimeout(2.5),
            Component.climber.c_gotoDown().withTimeout(2.5)

        );
    }

    public static Command c_shootLeft() {
        return c_shoot("left", true);
    }
    public static Command c_shootStraight() {
        return c_shoot("straight", true);
    }
    public static Command c_shootRight() {
        return c_shoot("right", true);
    }

    public static Command c_shootCenterLeft() {
        return c_shoot("center left", true);
    }
    public static Command c_shootCenterRight() {
        return c_shoot("center right", true);
    }

    public static Command c_shootAndClimbLeft() {
        return c_shootAndClimb("left");
    }
    public static Command c_shootAndClimbCenterLeft() {
        return c_shootAndClimb("center left");
    }

    public static Command c_climbCenterLeft() {
        return new AsyncPathPlannerSequence(
            async(Component.climber.c_gotoUp()),
            PathManager.c_path("climb"),
            Component.climber.c_gotoDown()
        );
    }

    public static Command c_climbTest() {
        return new AsyncPathPlannerSequence(
            async(Component.climber.c_gotoUp()),
            PathManager.c_path("climb test"),
            Component.climber.c_gotoDown()
        );
    }
    
    public static Command c_sams5StepPlanToWealthAndHappiness(){
        return new SequentialCommandGroup(
            Component.chassis.c_rotateTo(0.25, false).withTimeout(1.5),
            Component.chassis.c_driveRobotRelative(-2, 0, 0).withTimeout(1.5),
            Component.chassis.c_driveRobotRelative(0, 2, 0).withTimeout(2)
        );
    }

    public static Command c_dip(){
        return new AsyncPathPlannerSequence(
            PathManager.c_path("intake 1"),
            // Component.intake.c_extend(),
            async(Component.intake.c_intake().withTimeout(2))
        );
    }

}
