package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.standard.commands.AlwaysRunnableInstantCommand;
import org.usfirst.frc4904.standard.commands.RunUnless;
import org.usfirst.frc4904.standard.humaninput.Driver;

import static org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig.JOYSTICK_DEADZONE;
import static org.usfirst.frc4904.robot.subsystems.ShooterSubsystem.calcRobotAngle;

public class RuffyDriver extends Driver {

    private static final double SPEED_EXP = 2, TURN_EXP = 2; // TODO TUNE

    public RuffyDriver() {
        super("RuffySwerveGain");
    }

    protected double scaleGain(double input, double exp) {
        return MathUtil.copyDirectionPow(input, exp);
    }

    @Override
    public void bindCommands() {
        var xyJoystick = HumanInput.Driver.xyJoystick;
        var turnJoystick = HumanInput.Driver.turnJoystick;

        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(this::getTranslation, this::getTurnSpeed)
                .withName("Driver - swerve drive")
        );

        turnJoystick.button1.whileTrue(
            Component.chassis.c_rotateTo(() -> calcRobotAngle(ShooterSubsystem.getOwnHub().pos), true)
        );

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(
            new AlwaysRunnableInstantCommand(() -> Component.chassis.resetOdometry())
        );

        // no zeroing when enabled (prevent accidental mid-match zeroing)
        xyJoystick.button2.onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.zero()),
                DriverStation::isTeleopEnabled
            )
        );
        turnJoystick.button2.onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.flipZero()),
                DriverStation::isTeleopEnabled
            )
        );
    }

    @Override
    public void unbindCommands() {
        Component.chassis.removeDefaultCommand();
    }

    protected double getRawForward() {
        return -HumanInput.Driver.xyJoystick.getY();
    }
    protected double getRawLeft() {
        return -HumanInput.Driver.xyJoystick.getX();
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
        double turnSpeed = -HumanInput.Driver.turnJoystick.getX();
        return scaleGain(turnSpeed, TURN_EXP);
    }
}
