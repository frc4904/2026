package org.usfirst.frc4904.standard.util;

import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;

/**
 * Common utilities
 */
public final class Util {

    private Util() {
        throw new UnsupportedOperationException("Cannot instantiate utility class.");
    }

    /**
     * Returns true if {@code value} is less than {@code epsilon}. This is useful
     * for floating point numbers, whose arithmetic operations tend to introduce
     * small errors.
     *
     *
     * @param value   The floating point number to be compared
     * @param epsilon The maximum magnitude of var such that it can be considered
     *                zero
     * @return Whether {@code value} is less than {@code epsilon}
     */
    public static boolean isZero(double value, double epsilon) {
        return Math.abs(value) < epsilon;
    }

    /**
     * Returns true if {@code value} is less than {@code epsilon}. This is useful
     * for floating point numbers, whose arithmetic operations tend to introduce
     * small errors.
     *
     * @param value The floating point number to be compared
     * @return Whether {@code value} is effectively zero
     */
    public static boolean isZero(double value) {
        return isZero(value, Math.sqrt(Math.ulp(1.0)));
    }

    // TODO upgrade java version and use Math.clamp
    /**
     * Clamp a value between a minimum and maximum
     *
     * @param value The input value
     * @param min The minimum allowed value
     * @param max The maximum allowed value
     * @return The clamped value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Clamp a value between a minimum and maximum
     *
     * @param value The input value
     * @param min The minimum allowed value
     * @param max The maximum allowed value
     * @return The clamped value
     */
    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Clamp a value between a minimum and maximum
     *
     * @param value The input value
     * @param min The minimum allowed value
     * @param max The maximum allowed value
     * @return The clamped value
     */
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Transform a number from one range into another.
     * For example, transforming an input from [-1, 1] to [0, 10]:
     * <pre>{@code
     * transformRange(getInput(), -1, 1, 0, 10);
     * }</pre>
     * @param x Input number. Does <em>not</em> have to be within [fromMin, fromMax] and will <em>not</em> be clamped
     * @param fromMin Minimum value of the initial range
     * @param fromMax Maximum value of the initial range
     * @param toMin Minimum value of the target range
     * @param toMax Maximum value of the target range
     * @return The transformed value. Will be <em>outside</em> of [toMin, toMax] if the initial value was outside of [fromMin, fromMax]
     */
    public static double transformRange(double x, double fromMin, double fromMax, double toMin, double toMax) {
        return (x - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
    }

    /** See {@link #transformRange(double, double, double, double, double)} */
    public static double transformRange(double x, Range from, Range to) {
        return transformRange(x, from.min, from.max, to.min, to.max);
    }

    public static double lerp(double x, double a, double b) {
        return a + (b - a) * x;
    }

    public record Range(double min, double max) {
        public Range {
            if (min > max) {
                throw new BoundaryException("Range min " + min + " greater than max " + max);
            }
        }

        public double getRange() {
            return max - min;
        }

        public boolean contains(double value) {
            return value >= min && value <= max;
        }

        public double getCenter() {
            return (min + max) / 2.0;
        }

        /**
         * Scales a value (between -1 and 1) to the range. Example: (new Range(0,6)).scaleValue(0.5) == 4.5
         *
         * @param value between -1 and 1 (will be limited to [-1, 1])
         * @return the scaled value
         */
        public double scaleValue(double value) {
            return limitValue(getCenter() + value * (getRange() / 2.0));
        }

        /**
         * Limits a value to the range. Example: (new Range(0,6)).limitValue(7) == 6
         *
         * @param value the value to be limited
         * @return the limited value
         */
        public double limitValue(double value) {
            return Math.max(Math.min(value, max), min);
        }
    }

    /**
     * Computes the conversion factor between the first and second {@link TimeUnit}
     * given.
     *
     * @param from the source unit
     * @param to   the target unit
     * @return the conversion factor
     */
    public static double timeConversionFactor(TimeUnit from, TimeUnit to) {
        // TimeUnit.convert returns a long.
        // If from >= to (in terms of units), we can simply use normal conversion, as it
        // will scale up.
        // Otherwise, invert the conversion and take the reciprocal.

        if (from.compareTo(to) >= 0) {
            return (double) to.convert(1, from);
        } else {
            return 1.0 / from.convert(1, to);
        }
    }

    private static Double fpgaTimeOffset;

    public static double epochSecondsToFPGATimestamp(double seconds) {
        if (fpgaTimeOffset == null) {
            double now = System.currentTimeMillis() / 1000.0;
            fpgaTimeOffset = Timer.getFPGATimestamp() - now;
        }

        return seconds + fpgaTimeOffset;
    }

    public static double fahrenheit(double celsius) {
        return Celsius.of(celsius).in(Fahrenheit);
    }

    /**
     * Combines multiple boolean suppliers with a logical OR operator.
     *
     * @param conditions the boolean suppliers
     * @return callback which returns true if any of the suppliers return true
     */
    public static BooleanSupplier any(BooleanSupplier... conditions) {
        if (conditions.length == 1) return conditions[0];

        return () -> {
            for (var supplier : conditions) {
                if (supplier.getAsBoolean()) return true;
            }
            return false;
        };
    }

    /**
     * Combines multiple boolean suppliers with a logical AND operator.
     *
     * @param conditions the boolean suppliers
     * @return callback which returns true if all of the suppliers return true
     */
    public static BooleanSupplier all(BooleanSupplier... conditions) {
        if (conditions.length == 1) return conditions[0];

        return () -> {
            for (var supplier : conditions) {
                if (!supplier.getAsBoolean()) return false;
            }
            return true;
        };
    }

    /**
     * Utility for creating Transform2D instances
     */
    public static Transform2d transform2d(double x, double y, double rotations) {
        return new Transform2d(x, y, Rotation2d.fromRotations(rotations));
    }

    /**
     * Utility for creating Transform3D instances
     */
    public static Transform3d transform3d(double x, double y, double z, double radians) {
        return new Transform3d(x, y, z, new Rotation3d(0, 0, radians));
    }

    /**
     * Clears the pose(s) from {@link FieldObject2d} instances
     */
    public static void clearPose(FieldObject2d... poses) {
        for (var pose : poses) pose.setPoses();
    }

}
