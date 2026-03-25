package org.usfirst.frc4904.standard.custom.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.usfirst.frc4904.standard.util.Storage;

public class CustomDutyCycleEncoder extends DutyCycleEncoder {

    // TODO worst thing since sliced bread
    private static final double[] ZEROS = {
        0.607, // fl
        0.676, // fr
        0.361, // bl
        0.461  // br
    };

    private final String key;

    private double resetOffset;

    public CustomDutyCycleEncoder(int channel) {
        super(channel);

        key = "zeros/" + channel;

        resetOffset = Storage.getDouble(key, 0);
        // resetOffset = channel < ZEROS.length ? ZEROS[channel] : 0;
    }

    public void reset() {
        setResetOffset(super.get());
    }

    public void flip() {
        setResetOffset(resetOffset + 0.5);
    }

    public void setResetOffset(double offset) {
        offset = IMU.wrapAnglePositive(offset);

        resetOffset = offset;
        Storage.setDouble(key, offset);
    }

    public double getResetOffset() {
        return resetOffset;
    }

    @Override
    public double get() {
        double value = super.get() - resetOffset;
        return value < 0 ? value + 1 : value;
    }

    public double getAbsolute() {
        return super.get();
    }
}
