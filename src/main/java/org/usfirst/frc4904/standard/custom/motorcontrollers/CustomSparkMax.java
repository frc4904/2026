package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class CustomSparkMax extends SparkMax implements SmartMotorController {

    protected final boolean limitSwitch;

    public CustomSparkMax(int deviceNumber, MotorType motorType, boolean inverted, boolean limitSwitch) {
        super(deviceNumber, motorType);

        setInverted(inverted);
        this.limitSwitch = limitSwitch;
    }

    public CustomSparkMax(int deviceNumber, MotorType motorType, boolean inverted) {
        this(deviceNumber, motorType, inverted, false);
    }

    public CustomSparkMax(int deviceNumber, MotorType motorType) {
        this(deviceNumber, motorType, false, false);
    }

    private void configure(SparkBaseConfig config) {
        configure(
            config,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );
    }

    @Override
    public void setMotorBrake(boolean brake) {
        configure(new SparkMaxConfig().idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast));
    }

    @SuppressWarnings("deprecation") // overrides deprecated method
    @Override
    public void setInverted(boolean inverted) {
        configure(new SparkMaxConfig().inverted(inverted));
    }

    @Override
    public boolean isFwdLimitSwitchPressed() throws IllegalAccessException {
        if (!limitSwitch) throw new IllegalAccessException("Cannot read forward limit switch state when CustomSparkMax was constructed without limit switch.");
        return getForwardLimitSwitch().isPressed();
    }

    @Override
    public boolean isRevLimitSwitchPressed() throws IllegalAccessException {
        if (!limitSwitch) throw new IllegalAccessException("Cannot read reverse limit switch state when CustomSparkMax was constructed without limit switch.");
        return getReverseLimitSwitch().isPressed();
    }
}
