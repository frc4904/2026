package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.standard.commands.AlwaysRunnableInstantCommand;
import org.usfirst.frc4904.standard.commands.RunIf;
import org.usfirst.frc4904.standard.commands.RunUnless;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandPS4;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.Storage;

import static org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig.JOYSTICK_DEADZONE;

import java.util.prefs.Preferences;

public class SwerveDriver extends Driver {

    private static final double SPEED_EXP = 2, TURN_EXP = 2; // TODO TUNE

    public SwerveDriver() {
        super("SwerveDriver");
    }

    @Override
    public void bindCommands() {
        CustomCommandPS4 ps4 = HumanInput.Driver.ps4;

        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(this::getTranslation, this::getTurnSpeed)
                .withName("Driver - swerve drive")
        );

        // navx reset
        ps4.povUp().onTrue(
            new AlwaysRunnableInstantCommand(() -> Component.chassis.resetOdometry())
        );

        // flip zeroes
        ps4.povLeft().onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.flipZero()),
                DriverStation::isTeleopEnabled
            )
        );

        // swerve reset
        ps4.povRight().onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.zero()),
                DriverStation::isTeleopEnabled
            )
        );

        // ps4.povRight().onTrue(
        //     new AlwaysRunnableInstantCommand(() -> edu.wpi.first.wpilibj.Preferences.setDouble("cheese", 5))
        // );
        // ps4.povRight().whileTrue(
        //     Component.INTAKE_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.c_forward(true)
        // );

        // climber up
        ps4.triangle().onTrue(Component.climber.c_gotoUp());
        // climber down
        ps4.cross().onTrue(Component.climber.c_gotoDown());

        // long shoot
        ps4.circle().whileTrue(Operator.wrapShootCommand(Component.shooter.c_longShoot()));

        // indexer
        ps4.square().whileTrue(Component.indexer.c_forward(true));

        // intake retract
        ps4.L1().onTrue(Component.intake.c_retract());
        // intake extend
        ps4.L2().onTrue(Component.intake.c_extend());
        // intake wobble
        ps4.povDown().whileTrue(Component.intake.c_wobble());
        // run intake while any of the above are held
        ps4.L1().or(ps4.L2()).or(ps4.povDown()).whileTrue(Component.intake.c_intake());

        // align
        ps4.R1().whileTrue(ShooterSubsystem.c_smartShootAlign());

        // index and shooter
        ps4.R2().whileTrue(Operator.c_smartShoot());
        ps4.R2().whileFalse(
            new RunIf(
                Component.shooter.c_smartShoot().withTimeout(0.7),
                Component.shooter::canShoot
            )
        );
    }

    @Override
    public void unbindCommands() {
        Component.chassis.removeDefaultCommand();
    }

    protected double getRawForward() {
        return -HumanInput.Driver.ps4.getLeftY();
    }
    protected double getRawLeft() {
        return -HumanInput.Driver.ps4.getLeftX();
    }

    @Override
    public Translation2d getTranslation() {
        try {
            Translation2d translation = new Translation2d(getRawForward(), getRawLeft());
            double mag = translation.getNorm();
            if (mag == 0) return translation;

            double len = scaleGain(MathUtil.applyDeadband(mag, JOYSTICK_DEADZONE), SPEED_EXP);
            return translation.times(len / mag); // unit translation * len
        } catch (Exception e) {
            System.err.println("womp womp");
            return Translation2d.kZero;
        }
    }

    @Override
    public double getTurnSpeed() {
        try {
            double turnSpeed = -HumanInput.Driver.ps4.getRightX();
            return scaleGain(turnSpeed, TURN_EXP);
        } catch (Exception e) {
            System.err.println("womp womp");
            return 0;
        }
    }
}
