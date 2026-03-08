package org.usfirst.frc4904.standard.util.logging;
import org.usfirst.frc4904.standard.util.Logging;

public final class Frogging {
    public static void frog(String key, Object... values) {
        Logging.log(key, values);
    }
}

