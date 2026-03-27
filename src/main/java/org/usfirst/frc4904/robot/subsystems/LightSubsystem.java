package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.util.Perlin2D;
import org.usfirst.frc4904.standard.util.Util;

import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.Set;

public class LightSubsystem extends SubsystemBase {

    public static final class Color {

        public static final int[] SUCCESS  = { 0, 200, 50 };
        public static final int[] FAIL     = { 255, 50, 0 };

        public static final int[] AUTON    = { 127, 50, 255 };
        public static final int[] TELEOP   = { 255, 100, 0 };
        public static final int[] DISABLED = { 200, 220, 255 };

        private Color() {}

    }

    private float[] flashColor = new float[4];
    private float flashStrength = 0;

    double lastUpdateTime;

    private static class BufferViewData {

        public final float[][] colorArray;
        public final AddressableLEDBufferView view;

        public BufferViewData(AddressableLEDBuffer buffer, int start, int length, boolean reverse) {
            colorArray = new float[length][3];

            int end = start + length - 1;
            view = buffer.createView(reverse ? end : start, reverse ? start : end);
        }

        public void copyColorsToBuffer() {
            for (int i = 0; i < colorArray.length; i++) {
                float[] color = colorArray[i];

                view.setRGB(
                    i,
                    (int) (Math.pow(color[0], 2) * 255),
                    (int) (Math.pow(color[1], 2) * 255),
                    (int) (Math.pow(color[2], 2) * 255)
                );
            }
        }

    }

    final AddressableLED led;
    final AddressableLEDBuffer buffer;
    final BufferViewData[] views;

    public LightSubsystem(AddressableLED led, int ledLength, int[] lengths, boolean[] reverse) {
        this.led = led;
        buffer = new AddressableLEDBuffer(ledLength);

        led.setLength(ledLength);
        led.start();

        this.views = new BufferViewData[lengths.length];
        for (int i = 0, start = 0; i < lengths.length; i++) {
            this.views[i] = new BufferViewData(buffer, start, lengths[i], reverse[i]);
            start += lengths[i];
        }

        lastUpdateTime = Timer.getFPGATimestamp();
    }

    private void alphaBlend(float[][] colors, float[] c2, float mix) {
        float a = c2.length == 4 ? c2[3] * mix : mix;
        float r = c2[0] * a, g = c2[1] * a, b = c2[2] * a;
        float aInv = 1 - a;

        for (float[] c1 : colors) {
            c1[0] = r + c1[0] * aInv;
            c1[1] = g + c1[1] * aInv;
            c1[2] = b + c1[2] * aInv;
        }
    }

    // blends with existing colors
    private void progressBar(float[][] colors, float[] color, float progress) {
        int length = colors.length;

        float a = color.length == 4 ? color[3] : 1;
        float r = color[0], g = color[1], b = color[2];

        for (int i = 0; i < length; i++) {
            float[] c1 = colors[i];
            float strength = Util.clamp(progress * length - i, 0, 1) * a;

            if (strength >= 1) {
                c1[0] = r;
                c1[1] = g;
                c1[2] = b;
            } else if (strength > 0) {
                float strengthInv = 1 - strength;

                c1[0] = r * strength + c1[0] * strengthInv;
                c1[1] = g * strength + c1[1] * strengthInv;
                c1[2] = b * strength + c1[2] * strengthInv;
            }
        }
    }

    private final Perlin2D fireNoise = new Perlin2D(12345678987654321L);

    // overwrites existing colors
    private void fire(float[][] colors, boolean blue) {
        float time = (float) lastUpdateTime;

        for (int i = 0; i < colors.length; i++) {
            float[] color = colors[i];

            float height = 1 - (float) i / (colors.length - 1);
            float noise = fireNoise.noise(time, -i * 0.13f + time * 2);
            float strength = (float) Math.pow(noise, 2.5) * 1.3f + height - 0.55f;

            float a = Util.clamp(strength * 4, 0, 1);

            float r = a * 1;
            float g = a * Util.clamp(strength * 2 - 0.5f, 0, 1);
            float b = a * Util.clamp(strength * 4 - 3, 0, 1);

            color[0] = blue ? b : r;
            color[1] = g * 0.8f; // green LEDs are brighter
            color[2] = blue ? r : b;
        }
    }

    /**
     * Flash an RGB color for about a second. Color is an array of 3 (RGB) or 4 (RGBA) ints from 0-255.
     */
    public void flashColor(int[] color) {
        if (color.length == 3) {
            flashColor(color[0], color[1], color[2]);
        } else if (color.length == 4) {
            flashColor(color[0], color[1], color[2], color[3]);
        } else {
            System.err.println("LightSubsystem.flashColor(int[] color) must take an array of 3 or 4 ints for RGB or RGBA");
            flashColor(0, 0, 0);
        }
    }

    /**
     * Flash an RGB color for about a second. Colors are from 0-255.
     */
    public void flashColor(int r, int g, int b) {
        flashColor(r, g, b, 255);
    }

    /**
     * Flash an RGB color for about a second. Colors and alpha are from 0-255.
     */
    public void flashColor(int r, int g, int b, int a) {
        flashColor = new float[] { r / 255f, g / 255f, b / 255f, a / 255f };
        flashStrength = 1;
    }

    private final Set<ProgressBar> progressBars = new LinkedHashSet<>();

    public class ProgressBar {
        private final float[] color;
        private final boolean[] viewsEnabled;
        private float progress = 0;

        public ProgressBar(int[] color) {
            this(color, null);
        }

        public ProgressBar(int[] color, boolean[] viewsEnabled) {
            if (color.length != 3 && color.length != 4) {
                System.err.println("LightSubsystem.addProgressBar() must take an array of 3 or 4 ints for RGB or RGBA");
                this.color = new float[3];
            } else {
                this.color = new float[] {
                    color[0] / 255f,
                    color[1] / 255f,
                    color[2] / 255f,
                    color.length == 4 ? color[3] / 255f : 1
                };
            }

            if (viewsEnabled != null && viewsEnabled.length != views.length) {
                System.err.println("LightSubsystem.addProgressBar() enabled views must be the same length as number of LED buffer views");
                viewsEnabled = null;
            }
            if (viewsEnabled == null) {
                this.viewsEnabled = new boolean[views.length];
                Arrays.fill(this.viewsEnabled, true);
            } else {
                this.viewsEnabled = viewsEnabled;
            }


            progressBars.add(this);
        }

        public void setProgress(double progress) {
            this.progress = (float) progress;
        }

        public void delete() {
            progressBars.remove(this);
        }
    }

    private final LEDPattern rainbowPattern =
        LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.3), Units.Meters.of(1.0 / 60));

    @Override
    public void periodic() {
        double time = Timer.getFPGATimestamp();
        double deltaTime = time - lastUpdateTime;
        lastUpdateTime = time;

        if (DriverStation.isDisabled()) {
            rainbowPattern.applyTo(buffer);
        } else {
            for (int i = 0; i < views.length; i++) {
                var view = views[i];
                float[][] colors = view.colorArray;

                fire(colors, DriverStation.isAutonomous());

                for (var bar : progressBars) {
                    if (bar.viewsEnabled[i] && bar.progress > 0) {
                        progressBar(colors, bar.color, bar.progress);
                    }
                }

                if (flashStrength > 0) {
                    alphaBlend(colors, flashColor, (float) Math.sqrt(flashStrength));
                    flashStrength -= (float) deltaTime / 1.5f;
                }

                view.copyColorsToBuffer();
            }
        }

        led.setData(buffer);
    }

}
