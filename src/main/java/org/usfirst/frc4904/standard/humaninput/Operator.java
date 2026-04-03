package org.usfirst.frc4904.standard.humaninput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.SwitchingIfElseCommand;
import org.usfirst.frc4904.standard.util.CmdUtil;

/**
 * Operator specific version of HumanInterface
 */
public abstract class Operator extends HumanInput {

    public Operator(String name) {
        super(name);
    }

    public static final double SHOOT_INDEXER_DELAY = 0.5;

    public static Command wrapShootCommand(Command command) {
        return new ParallelCommandGroup(
            command,
            CmdUtil.delayed(SHOOT_INDEXER_DELAY, new ParallelCommandGroup(
                Component.indexer.c_forward(true).asProxy(),
                Component.intake.c_wobble().asProxy()
            ))
        );
    }

    public static Command c_smartShoot() {
        return new SwitchingIfElseCommand(
            wrapShootCommand(Component.shooter.c_smartShoot()),
            null,
            Component.shooter::canShoot
        );
    }

}
