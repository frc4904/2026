package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick.Axis;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.Util;

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
        double velocity = Component.shooter.getShooterVelocityForDistance(distance);

        return velocity;
    }

    @Override
    public void bindCommands() {
        var joystick = RobotMap.HumanInput.Operator.joystick;

        /// INTAKE
        joystick.button5.onTrue(Component.intake.c_retract());

        joystick.button3.onTrue(Component.intake.c_extend());
        joystick.button3.whileTrue(Component.intake.c_intake());

        joystick.button4.whileTrue(Component.TEMPORARY_INTAKE_SHENANIGANS.c_forward(true));
        joystick.button6.whileTrue(Component.TEMPORARY_INTAKE_SHENANIGANS.c_backward(true));

        /// SHOOTER
        joystick.button1.whileTrue(Component.shooter.c_smartShoot());
        joystick.button2.whileTrue(Component.shooter.c_controlVelocity(this::getVelocity));

        /// CLIMBER
        joystick.button7.onTrue(Component.climber.c_down());
        joystick.button8.onTrue(Component.climber.c_up());

        /// INDEXER
        joystick.button9.whileTrue(Component.indexer.c_forward());

        /// NOTIFS TEST
        // joystick.button10.onTrue(Notifications.c_testNotif());
        // joystick.button11.onTrue(Notifications.c_sendRandom());

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
