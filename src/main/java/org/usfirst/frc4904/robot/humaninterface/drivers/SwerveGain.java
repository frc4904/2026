package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.standard.commands.AlwaysRunnableInstantCommand;
import org.usfirst.frc4904.standard.commands.RunUnless;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandPS4;
import org.usfirst.frc4904.standard.humaninput.Driver;

import static org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig.JOYSTICK_DEADZONE;
import static org.usfirst.frc4904.robot.subsystems.ShooterSubsystem.calcRobotAngle;

public class SwerveGain extends Driver {

    private static final double SPEED_EXP = 2, TURN_EXP = 2; // TODO TUNE

    public SwerveGain() {
        super("SwerveGain");
    }

    protected double scaleGain(double input, double exp) {
        return MathUtil.copyDirectionPow(input, exp);
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

        // explode operator joystick (shooter testing)
        ps4.povRight().whileTrue(Component.shooter.c_forward(true));

        // swerve reset
        ps4.povDown().onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.zero()),
                DriverStation::isTeleopEnabled
            )
        );

        // climber up
        ps4.triangle().whileTrue(Component.climber.c_up());
        // climber down
        ps4.cross().whileTrue(Component.climber.c_down());

        // testing shooter
        ps4.square().whileTrue(
            Component.shooter.c_controlVelocity(() -> ShooterSubsystem.getShooterVelocityForDistance(2))
        );

        // indexer
        ps4.circle().whileTrue(Component.indexer.c_forward(true));

        // intake retract
        ps4.L1().onTrue(Component.intake.c_retract());
        // intake extend and run
        ps4.L2().onTrue(Component.intake.c_extend());
        ps4.L2().whileTrue(Component.intake.c_intake());

        // align
        ps4.R1().whileTrue(
            Component.chassis.c_rotateTo(() -> calcRobotAngle(ShooterSubsystem.getOwnHub().pos))
        );

        // (index and) shooter
        ps4.R2().whileTrue(Component.shooter.c_smartShoot());
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
        Translation2d translation = new Translation2d(getRawForward(), getRawLeft());
        double mag = translation.getNorm();
        if (mag == 0) return translation;

        double len = scaleGain(MathUtil.applyDeadband(mag, JOYSTICK_DEADZONE), SPEED_EXP);
        return translation.times(len / mag); // unit translation * len
    }

    @Override
    public double getTurnSpeed() {
        double turnSpeed = -HumanInput.Driver.ps4.getRightX();
        return scaleGain(turnSpeed, TURN_EXP);
    }
}
