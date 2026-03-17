package org.usfirst.frc4904.standard.silly;
import org.usfirst.frc4904.standard.util.Logging;

public final class console {
    public static void log(String key) {
        Logging.log(key);
    }
    public static void logWithDelay(String key, double delaySeconds) {
        Logging.logWithDelay(key, delaySeconds);
    }
    public static <T> T log(String key, T value, Object... others) {
        return Logging.log(key, value, others);
    }
    public static <T> T logWithDelay(String key, double delaySeconds, T value, Object... others) {
        return Logging.logWithDelay(key, delaySeconds, value, others);
    }
}
