package org.usfirst.frc4904.standard.humaninput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Driver specific version of HumanInterface.
 */
public abstract class Driver extends HumanInput {

    public Driver(String name) {
        super(name);
    }

    /**
     * @return Speed that the driver wants, in wpilib field-relative coordinates (forward, left).
     *         Length of translation vector should not exceed 1
     */
    public abstract Translation2d getTranslation();

    /**
     * @return Turn speed that the driver wants in the range [-1, 1], positive = counterclockwise
     */
    public abstract double getTurnSpeed();

    /** Alias for {@link MathUtil#copyDirectionPow(double, double)} */
    protected double scaleGain(double input, double exp) {
        return MathUtil.copyDirectionPow(input, exp);
    }

}
