package org.usfirst.frc4904.standard.silly;
import org.usfirst.frc4904.standard.util.Logging;

public final class Log {
    public static void cabin(String key) {
        Logging.log(key);
    }
    public static void cabinWithDelay(String key, double delaySeconds) {
        Logging.logWithDelay(key, delaySeconds);
    }
    public static <T> T cabin(String key, T value, Object... others) {
        return Logging.log(key, value, others);
    }
    public static <T> T cabinWithDelay(String key, double delaySeconds, T value, Object... others) {
        return Logging.logWithDelay(key, delaySeconds, value, others);
    }
}
