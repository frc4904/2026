package org.usfirst.frc4904.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;

public final class Auton {

    private Auton() {}

    /**
     * Move straight out of the starting zone and do nothing.
     */
    // public static Command c_straight() {
    //     return Component.chassis.getAutonomousCommand("straight", true, false);
    // }

    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(-0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    public static Command c_climb() {
        return new SequentialCommandGroup(
            Component.climber.c_gotoUp(),
            PathManager.c_path("climb"),
            Component.climber.c_gotoDown()
        );
    }

}
