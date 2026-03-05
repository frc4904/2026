package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class CustomTalonFX extends TalonFX implements SmartMotorController {

    public CustomTalonFX(int deviceId, boolean inverted) {
        super(deviceId);
        setInverted(inverted);
    }

    public CustomTalonFX(int deviceId) {
        this(deviceId, false);
    }

    /**
     * Setting to toggle brake mode on neutral.
     * <p>
     * This does not brake the motor. Use {@code .neutralOutput()} instead, after {@code .setMotorBrake(true)}.
     */
    @Override
    public void setMotorBrake(boolean brake) {
        setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    // TODO: also support normally closed limit switches
    @Override
    public boolean isFwdLimitSwitchPressed() {
        return getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }
    @Override
    public boolean isRevLimitSwitchPressed() {
        // TODO: this boolean might be reversed
        return getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    private boolean inverted = false;

    /**
     * @param inverted Whether to invert - true is clockwise positive, false is counterclockwise positive (default).
     */
    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        // CounterClockwise_Positive seems to be the default value
        InvertedValue direction = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        getConfigurator().apply(new MotorOutputConfigs().withInverted(direction));
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

}
