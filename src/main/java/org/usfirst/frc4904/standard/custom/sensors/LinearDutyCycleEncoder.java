package org.usfirst.frc4904.standard.custom.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.usfirst.frc4904.standard.util.Storage;

/**
 * Converts a DutyCycleEncoder into encoding a linear position
 * by keeping track of the number of full revolutions.
 */
public class LinearDutyCycleEncoder {

    private final String resetOffsetKey, lastReadingKey;
    public final DutyCycleEncoder encoder;

    private double resetOffset;
    Double lastReading;

    public LinearDutyCycleEncoder(int channel) {
        resetOffsetKey = "zeros/" + channel;
        lastReadingKey = "lastReading/" + channel;
        encoder = new DutyCycleEncoder(channel);

        resetOffset = Storage.getDouble(resetOffsetKey, 0);

        if (Storage.has(lastReadingKey)) {
            lastReading = Storage.getDouble(lastReadingKey, 0);
        }
    }

    public void reset() {
        resetOffset = encoder.get();
        Storage.setDouble(resetOffsetKey, resetOffset);
    }

    public double get() {
        double reading = encoder.get();
        Storage.setDouble(lastReadingKey, reading);

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

            Storage.setDouble(resetOffsetKey, resetOffset);
        }

        lastReading = reading;

        return reading - resetOffset;
    }
}
