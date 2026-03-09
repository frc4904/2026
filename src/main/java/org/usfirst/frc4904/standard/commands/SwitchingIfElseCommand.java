package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc4904.standard.util.CmdUtil;
import org.usfirst.frc4904.standard.util.Util;

import java.util.function.BooleanSupplier;

public class SwitchingIfElseCommand extends Command {

    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    /**
     * Similar to {@link RunIfElse}, but will switch which command is
     * running mid-execution if the supplier changes.
     */
    public SwitchingIfElseCommand(Command onTrue, Command onFalse, BooleanSupplier... conditions) {
        this.onTrue = CmdUtil.nonNull(onTrue);
        this.onFalse = CmdUtil.nonNull(onFalse);
        this.condition = Util.all(conditions);

        if (onTrue != null) {
            CommandScheduler.getInstance().registerComposedCommands(onTrue);
            addRequirements(onTrue.getRequirements());
        }
        if (onFalse != null) {
            CommandScheduler.getInstance().registerComposedCommands(onFalse);
            addRequirements(onFalse.getRequirements());
        }
    }

    private Command getCurrent() {
        return condition.getAsBoolean() ? onTrue : onFalse;
    }

    private Command current;

    @Override
    public void initialize() {
        current = getCurrent();
    }

    @Override
    public void execute() {
        Command cmd = getCurrent();
        if (cmd != current) {
            current.cancel();
            cmd.initialize();
            current = cmd;
        }
        current.execute();
    }

    // below methods are the same as wpilib ConditionalCommand

    @Override
    public void end(boolean interrupted) {
        current.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return current.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (onTrue.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
            || onFalse.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            return InterruptionBehavior.kCancelSelf;
        } else {
            return InterruptionBehavior.kCancelIncoming;
        }
    }
}
