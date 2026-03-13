package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick.Axis;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.Util;

import static org.usfirst.frc4904.robot.subsystems.ShooterSubsystem.getShooterVelocityForDistance;

public class DefaultOperator extends Operator {

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    double getVelocity() {
        var joystick = RobotMap.HumanInput.Operator.joystick;

        double value = joystick.getAxis(Axis.SLIDER);
        double distance = Util.transformRange(value, 1, -1, 2, 5);
        double velocity = getShooterVelocityForDistance(distance);

        return velocity;
    }

    @Override
    public void bindCommands() {
        var joystick = RobotMap.HumanInput.Operator.joystick;

        /// TEMPORARY INTAKE SHENANIGANS
        // joystick.button4.whileTrue(Component.INTAKE_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.c_forward(true));
        // joystick.button6.whileTrue(Component.INTAKE_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.c_backward(true));

        // joystick.button10.whileTrue(
        //     Component.TEMPORARY_INTAKE_SHENANIGANS.c_DEBUG_tunePIDkG(() -> 1 - joystick.getAxis(Axis.SLIDER))
        // );

        /// INTAKE
        joystick.button3.onTrue(Component.intake.c_extend());
        joystick.button5.onTrue(Component.intake.c_retract());
        joystick.button11.whileTrue(Component.intake.c_wobble());

        joystick.button3.or(joystick.button5).or(joystick.button11)
            .whileTrue(Component.intake.c_intake());

        /// INDEXER
        joystick.button4.whileTrue(Component.indexer.c_backward(true));
        joystick.button6.whileTrue(Component.indexer.c_forward(true));

<<<<<<< HEAD
        /// SHOOTER
        joystick.button1.whileTrue(wrapShootCommand(Component.shooter.c_smartShoot()));
        joystick.button2.whileTrue(wrapShootCommand(Component.shooter.c_longShoot()));
        joystick.button12.whileTrue(
            wrapShootCommand(Component.shooter.c_controlVelocity(this::getVelocity))
=======
        joystick.button11.whileTrue(Component.intake.c_wobble());

        /// SHOOTER
        joystick.button1.whileTrue(c_smartShootAndIndex());
        joystick.button2.whileTrue(Component.shooter.c_longShoot());
        joystick.button12.whileTrue(
            new ParallelCommandGroup(
                Component.shooter.c_controlVelocity(this::getVelocity),
                CmdUtil.delayed(SHOOT_INDEXER_DELAY, Component.indexer.c_forward(true))
            )
>>>>>>> 565cf6d (idk)
        );

        /// CLIMBER
        joystick.button9.whileTrue(Component.climber.c_down());
        joystick.button10.whileTrue(Component.climber.c_up());

        /// NOTIFS
        // joystick.button7.onTrue(Notifications.c_sendRandom());

        /// ORCHESTRA
        // CustomTalonFX[] motors = {
        //     Component.flDrive,
        //     Component.frDrive,
        //     Component.blDrive,
        //     Component.brDrive,
        //     Component.flTurn,
        //     null, null, null,
        //     Component.frTurn,
        //     null, null, null,
        //     Component.blTurn,
        //     null, null, null,
        //     Component.brTurn
        // };

        // joystick.button7.onTrue(OrchestraSubsystem.c_loadAndPlaySong(
        //     "shreksophone",
        //     4,
        //     motors
        // ));
        // joystick.button8.onTrue(OrchestraSubsystem.c_loadAndPlaySong(
        //     "coconutNyoom",
        //     4,
        //     motors
        // ));

        // joystick.button12.onTrue(new InstantCommand(OrchestraSubsystem::stopAll));
    }
}
