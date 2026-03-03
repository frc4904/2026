package org.usfirst.frc4904.standard.custom.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;

public class CustomDutyCycleEncoder extends DutyCycleEncoder {

    private final String key;

    private double resetOffset;

    public CustomDutyCycleEncoder(int channel) {
        super(channel);

        key = "zeros/" + channel;

        resetOffset = Preferences.getDouble(key, 0);
    }

    public void reset() {
        setResetOffset(super.get());
    }

    public void flip() {
        setResetOffset(resetOffset + 0.5);
    }

    public void setResetOffset(double offset) {
        offset %= 1;
        if (offset < 0) offset++;

        resetOffset = offset;
        Preferences.setDouble(key, offset);
    }

    public double getResetOffset() {
        return resetOffset;
    }

    @Override
    public double get() {
        double value = super.get() - resetOffset;
        return value < 0 ? value + 1 : value;
    }
}