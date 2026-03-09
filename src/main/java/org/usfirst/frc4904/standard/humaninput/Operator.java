package org.usfirst.frc4904.standard.humaninput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.SwitchingIfElseCommand;

/**
 * Operator specific version of HumanInterface
 */
public abstract class Operator extends HumanInput {

    public Operator(String name) {
        super(name);
    }

    public static final double SHOOT_INDEXER_DELAY = 0.25;

    public static Command c_smartShootAndIndex() {
        return new SwitchingIfElseCommand(
            new ParallelCommandGroup(
                Component.shooter.c_smartShoot(),
                new WaitCommand(SHOOT_INDEXER_DELAY).andThen(Component.indexer.c_forward(true))
            ),
            null,
            Component.shooter::canShoot
        );
    }

}
