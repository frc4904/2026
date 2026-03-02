package org.usfirst.frc4904.standard.custom.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Converts a DutyCycleEncoder into encoding a linear position
 * by keeping track of the number of full revolutions.
 */
public class LinearDutyCycleEncoder {

    private final String key;
    public final DutyCycleEncoder encoder;

    private double resetOffset;

    Double lastReading;

    public LinearDutyCycleEncoder(int channel) {
        key = "zeros/" + channel;
        encoder = new DutyCycleEncoder(channel);

        Preferences.initDouble(key, 0);
        resetOffset = Preferences.getDouble(key, 0);
    }

    public void reset() {
        resetOffset = encoder.get();
        Preferences.setDouble(key, resetOffset);
    }

    public double get() {
        double reading = encoder.get();

        if (lastReading == null) {
            lastReading = reading;
            return reading - resetOffset;
        }

        if (Math.abs(reading - lastReading) >= 0.5) {
            if (reading < lastReading) {
                resetOffset--;
            } else {
                resetOffset++;
            }

            Preferences.setDouble(key, resetOffset);
        }

        lastReading = reading;

        return reading - resetOffset;
    }
}
