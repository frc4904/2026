package org.usfirst.frc4904.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.auton.PathManager.SequentialPathPlannerGroup;
import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.standard.util.CmdUtil;

public final class Auton {
    private Auton() {}

    /// JANK

    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(3, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(-3, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    /// PATHPLANNER

    private static Command c_shoot(String type, boolean extraTime) {
        return new SequentialPathPlannerGroup(
            CmdUtil.asInstant(Component.intake.c_extend()),
            PathManager.c_path("shoot " + type),
            new ParallelCommandGroup(
                Component.shooter.c_smartShoot(),
                ShooterSubsystem.c_smartShootAlign(),
                Component.indexer.c_forward(true),
                Component.intake.c_intake(),
                Component.intake.c_wobble(),
                // after some time, put the intake up
                CmdUtil.delayed(extraTime ? 6 : 4, CmdUtil.asInstant(Component.intake.c_retract()))
            ).withTimeout(extraTime ? 15 : 8)
        );
    }

    private static Command c_shootAndClimb(String type) {
        return new SequentialPathPlannerGroup(
            CmdUtil.asInstant(Component.climber.c_gotoUp()),
            c_shoot(type, false),
            PathManager.c_path("climb from shoot " + type),
            Component.climber.c_gotoDown()
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
        return new SequentialPathPlannerGroup(
            Component.climber.c_gotoUp(),
            PathManager.c_path("climb"),
            Component.climber.c_gotoDown()
        );
    }

}
