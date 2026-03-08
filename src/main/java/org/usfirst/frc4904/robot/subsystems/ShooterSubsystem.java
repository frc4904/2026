package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import org.usfirst.frc4904.robot.subsystems.IO.ShootIO;
import org.usfirst.frc4904.robot.subsystems.IO.ShootSimIO;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends MotorSubsystem {

    private static final Slot0Configs pidfConfig = new Slot0Configs();
    static {
        // TODO tune
        pidfConfig.kP = 1;
        pidfConfig.kI = 0;
        pidfConfig.kD = 0;
        pidfConfig.kS = 0;
        pidfConfig.kV = 0;
        pidfConfig.kA = 0;
        pidfConfig.kG = 0;
    }

    public ShooterSubsystem(ShootSimIO shootIO) {
        super(
           ShootSimIO.
        );

        shooterMotor1.getConfigurator().apply(pidfConfig);
        shooterMotor2.getConfigurator().apply(pidfConfig);
    }

    public Command c_basicShoot() {
        return c_forward(true);
    }

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Command c_controlVelocity(DoubleSupplier getVelocity) {
        return runEnd(() -> {
            velocityRequest.withVelocity(getVelocity.getAsDouble());
            for (var motor : motors) {
                ((CustomTalonFX) motor).setControl(velocityRequest);
            }
        }, this::stop);
    }

}
