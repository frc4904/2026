package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        joystick.button4.whileTrue(Component.TEMPORARY_INTAKE_SHENANIGANS.c_forward(true));
        joystick.button6.whileTrue(Component.TEMPORARY_INTAKE_SHENANIGANS.c_backward(true));

        // joystick.button10.whileTrue(
        //     Component.TEMPORARY_INTAKE_SHENANIGANS.c_DEBUG_tunePIDkG(() -> 1 - joystick.getAxis(Axis.SLIDER))
        // );

        /// INTAKE
        joystick.button3.onTrue(Component.intake.c_extend());
        joystick.button5.onTrue(Component.intake.c_retract());

        joystick.button3.or(joystick.button5).whileTrue(Component.intake.c_intake());
        // joystick.button3.whileTrue(Component.intake.c_intake());


        /// SHOOTER
        joystick.button1.whileTrue(c_smartShootAndIndex());
        joystick.button2.whileTrue(
            new ParallelCommandGroup(
                Component.shooter.c_controlVelocity(this::getVelocity),
                new WaitCommand(SHOOT_INDEXER_DELAY).andThen(Component.indexer.c_forward(true))
            )
        );

        joystick.button11.whileTrue(Component.shooter.c_forward(true));

        /// CLIMBER
        joystick.button7.whileTrue(Component.climber.c_down());
        joystick.button8.whileTrue(Component.climber.c_up());

        /// INDEXER
        joystick.button9.whileTrue(Component.indexer.c_forward(true));
        joystick.button10.whileTrue(Component.indexer.c_backward(true));


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
