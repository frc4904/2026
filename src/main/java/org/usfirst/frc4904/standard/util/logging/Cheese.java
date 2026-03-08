package org.usfirst.frc4904.standard.util.logging;
import org.usfirst.frc4904.standard.util.Logging;

public final class Cheese {
    public static void tax(String key, Object... values) {
        Logging.log(key, values);
    }
}

