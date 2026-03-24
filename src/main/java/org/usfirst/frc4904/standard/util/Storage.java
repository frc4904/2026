package org.usfirst.frc4904.standard.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

/**
 * Storage for values that need to persist on robot restart.
 * Values are stored in plain text in the format
 * <pre>{@code
 * key1
 * Tval1
 * key2
 * Tval2
 * ...
 * }</pre>
 * where keyN is the key passed to getX()/setX(), T is a char representing
 * the type of the value, and valN is the stringified value.
 */
public final class Storage {
    private Storage() {}

    private static final Path PATH = Paths.get("/home/lvuser/storage.txt");

    private static final char BYTE = 'b', SHORT = 's', INT = 'i', LONG = 'l', FLOAT = 'f', DOUBLE = 'd', BOOLEAN = 'B', STRING = 'S';

    private record Item(char type, Object value, String stringified) {}

    private static final Map<String, Item> map = new HashMap<>();
    private static boolean modified = false;

    private static boolean initialized = false;

    public static void init() {
        if (initialized) {
            throw new IllegalStateException("Storage already initialized");
        }
        initialized = true;

        if (!Files.exists(PATH)) return;

        long start = System.currentTimeMillis();

        try {
            String str = Files.readString(PATH);
            String[] lines = str.split("\n");

            // generated files have a trailing blank line - ignore it
            for (int i = 0; i < lines.length - 1; i += 2) {
                String key = lines[i], value = lines[i + 1];
                char type = value.charAt(0);

                map.put(key, new Item(type, parse(type, value.substring(1)), value));
            }
        } catch (IOException e) {
            throw new RuntimeException("Failed to initialize storage: " + e.getMessage(), e);
        } finally {
            long time = System.currentTimeMillis() - start;
            System.out.println("Storage.init(): " + time + "ms");
        }
    }

    public static void save() {
        if (!modified) return;
        modified = false;

        long start = System.currentTimeMillis();

        var sb = new StringBuilder();

        for (var entry : map.entrySet()) {
            sb.append(entry.getKey())
              .append("\n")
              .append(entry.getValue().stringified)
              .append("\n");
        }

        try {
            Files.writeString(PATH, sb.toString());
        } catch (IOException e) {
            throw new RuntimeException("Failed to save storage: " + e.getMessage(), e);
        } finally {
            long time = System.currentTimeMillis() - start;
            if (time >= 4) {
                System.out.println("Storage.save(): " + time + "ms");
            }
        }
    }

    private static Object parse(char type, String value) {
        return switch (type) {
            case BYTE -> Byte.parseByte(value);
            case SHORT -> Short.parseShort(value);
            case INT -> Integer.parseInt(value);
            case LONG -> Long.parseLong(value);
            case FLOAT -> Float.parseFloat(value);
            case DOUBLE -> Double.parseDouble(value);
            case BOOLEAN -> Boolean.parseBoolean(value);
            case STRING -> unescape(value);
            default -> throw new IllegalArgumentException("Unrecognized type: " + type);
        };
    }

    private static void set(String key, char type, Object val, String stringified) {
        if (key.indexOf('\n') != -1) {
            throw new IllegalArgumentException("Storage keys cannot contain newline characters (bad key: '" + key + "')");
        }

        modified = true;
        map.put(key, new Item(type, val, type + stringified));
    }

    public static void remove(String key) {
        modified = true;
        map.remove(key);
    }

    public static boolean has(String key) {
        return map.containsKey(key);
    }

    public static byte getByte(String key, byte backup) {
        Item val = map.get(key);
        return val != null && val.type == BYTE ? (byte) val.value : backup;
    }
    public static short getShort(String key, short backup) {
        Item val = map.get(key);
        return val != null && val.type == SHORT ? (short) val.value : backup;
    }
    public static int getInt(String key, int backup) {
        Item val = map.get(key);
        return val != null && val.type == INT ? (int) val.value : backup;
    }
    public static long getLong(String key, long backup) {
        Item val = map.get(key);
        return val != null && val.type == LONG ? (long) val.value : backup;
    }
    public static float getFloat(String key, float backup) {
        Item val = map.get(key);
        return val != null && val.type == FLOAT ? (float) val.value : backup;
    }
    public static double getDouble(String key, double backup) {
        Item val = map.get(key);
        return val != null && val.type == DOUBLE ? (double) val.value : backup;
    }
    public static boolean getBool(String key, boolean backup) {
        Item val = map.get(key);
        return val != null && val.type == BOOLEAN ? (boolean) val.value : backup;
    }
    public static String getString(String key, String backup) {
        Item val = map.get(key);
        return val != null && val.type == STRING ? (String) val.value : backup;
    }

    public static void setByte(String key, byte val) {
        set(key, BYTE, val, Byte.toString(val));
    }
    public static void setShort(String key, short val) {
        set(key, SHORT, val, Short.toString(val));
    }
    public static void setInt(String key, int val) {
        set(key, INT, val, Integer.toString(val));
    }
    public static void setLong(String key, long val) {
        set(key, LONG, val, Long.toString(val));
    }
    public static void setFloat(String key, float val) {
        set(key, FLOAT, val, Float.toString(val));
    }
    public static void setDouble(String key, double val) {
        set(key, DOUBLE, val, Double.toString(val));
    }
    public static void setBool(String key, boolean val) {
        set(key, BOOLEAN, val, Boolean.toString(val));
    }
    public static void setString(String key, String val) {
        set(key, STRING, val, escape(val));
    }

    private static String escape(String str) {
        // backslashes are escaped as \s instead of \\
        // so that \\n doesn't get converted \\n -> \n -> <newline> or \\n -> \<newline>
        return str.replaceAll("\\\\", "\\\\s").replaceAll("\n", "\\\\n");
    }
    private static String unescape(String str) {
        return str.replaceAll("\\\\s", "\\\\").replaceAll("\\\\n", "\n"); // this is why we need regex literals
    }
}
