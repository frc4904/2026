package org.usfirst.frc4904.standard.util;

import edu.wpi.first.math.geometry.*;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;

public final class Logging {

    private Logging() {
        throw new UnsupportedOperationException("Cannot instantiate utility class.");
    }

    private static final Map<String, Double> lastLogTimes = new HashMap<>();

    /**
     * @return True if at least {@code delaySeconds} seconds have passed since this function
     *         was last called with the same {@code key}.
     */
    public static boolean cooldown(String key, double delaySeconds) {
        double now = System.currentTimeMillis() / 1000.0;
        double prev = lastLogTimes.getOrDefault(key, Double.NEGATIVE_INFINITY);

        if (now - prev >= delaySeconds) {
            lastLogTimes.put(key, now);
            return true;
        } else return false;
    }

    /**
     * Log values with a cooldown to not flood the RioLog™. Formatted as 'key: value' or 'key: [val1, val2, ...]'
     *
     * @param key    Key that is used to determine the cooldown since the last message with the same key was sent
     * @param values Value(s) to log
     * @return Whether the value was logged or skipped on cooldown
     */
    public static boolean log(String key, Object... values) {
        return logWithDelay(key, 0.5, values);
    }

    /**
     * Log values with a cooldown to not flood the RioLog™. Formatted as 'key: value'
     *
     * @param key          Key that is used to determine the cooldown since the last message with the same key was sent
     * @param delaySeconds Minimum cooldown between logs
     * @param values       Value(s) to log
     * @return Whether the value was logged or skipped on cooldown
     */
    public static boolean logWithDelay(String key, double delaySeconds, Object... values) {
        if (!cooldown(key, delaySeconds)) return false;

        System.out.println(key + ": " + formatValue(values, false));
        return true;
    }

    private static String formatValue(Object value, boolean arrayBrackets) {
        if (value == null) {
            return "null";
        } else if (value.getClass().isArray()) {
            var sb = new StringBuilder(arrayBrackets ? "[" : "");
            int len = Array.getLength(value);
            for (int i = 0; i < len; i++) {
                if (i > 0) sb.append(", ");
                sb.append(formatValue(Array.get(value, i), true));
            }
            if (arrayBrackets) sb.append("]");
            return sb.toString();
        } else if (value instanceof Pose2d pose) {
            return String.format("Pose2d(X: %.2f, Y: %.2f, Rot: %.1fdeg)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        } else if (value instanceof Translation2d trns) {
            return String.format("Translation2d(X: %.2f, Y: %.2f, Rot: %.1fdeg)", trns.getX(), trns.getY(), trns.getAngle().getDegrees());
        } else if (value instanceof Rotation3d rot) {
            return String.format("Rotation3d(%s)", rot3dToString(rot));
        } else if (value instanceof Pose3d pose) {
            return String.format("Pose3d(X: %.2f, Y: %.2f, Z: %.2f, %s)", pose.getX(), pose.getY(), pose.getZ(), rot3dToString(pose.getRotation()));
        } else if (value instanceof Transform3d trns) {
            return String.format("Pose3d(X: %.2f, Y: %.2f, Z: %.2f, %s)", trns.getX(), trns.getY(), trns.getZ(), rot3dToString(trns.getRotation()));
        } else {
            return value.toString();
        }
    }

    private static String rot3dToString(Rotation3d rot) {
        double x = rot.getMeasureX().in(Degrees);
        double y = rot.getMeasureY().in(Degrees);
        double z = rot.getMeasureZ().in(Degrees);

        return x == 0 && y == 0
            ? String.format("Yaw: %.1fdeg", z)
            : String.format("X/Roll: %.1fdeg, Y/Pitch: %.1fdeg, Z/Yaw: %.1fdeg", x, y, z);
    }
}
