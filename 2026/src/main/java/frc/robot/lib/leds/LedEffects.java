package frc.robot.lib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * LED effects implementation. Owned by LedController (composition).
 * Preallocates buffer and timers; methods are optimized to avoid per-frame allocations.
 */
public class LedEffects {
    private final AddressableLED m_Led;
    private final AddressableLEDBuffer ledBuffer;

    private final Timer strobeTimer = new Timer();
    private final Timer zipTimer = new Timer();
    private final Timer fillTimer = new Timer();
    private final Timer carnivalTimer = new Timer();

    private volatile int brightnessLimit;
    private int m_rainbowFirstPixelHue;
    private int m_waveValue;
    private int m_range;
    private int zipIncrease;
    private int start;
    private int end;
    private static final double fadeExponent = 0.4;
    public boolean on = true;
    public boolean starter = true;

    public LedEffects(int length, int port, int brightness) {
        this.brightnessLimit = brightness;
        m_Led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        m_Led.setLength(ledBuffer.getLength());
        m_Led.setData(ledBuffer);
        m_Led.start();
        strobeTimer.start();
        zipTimer.start();
        carnivalTimer.start();
        fillTimer.start();
    }

    /** Expose setData for owning controller to push the buffer to hardware. */
    public void setData() {
        m_Led.setData(ledBuffer);
    }

    public void setBrightness(int brightness) {
        this.brightnessLimit = brightness;
    }

    public int getLedBrightness() {
        return brightnessLimit;
    }

    // Helper: scale an intrinsic color value (0..255) by controller brightness (0..255)
    // result stays in 0..255, uses integer math to avoid allocations.
    private int applyBrightness(int colorValue) {
        return (int) ((colorValue * (double) brightnessLimit) / 255L);
    }

    /* --- Effects (use applyBrightness for V) --- */

    public void solid(String section, LedColor color) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int hue = color.hues();
        final int value = applyBrightness(color.value());
        for (int i = s; i < e; i++) {
            ledBuffer.setHSV(i, hue, 255, value);
        }
    }

    public void solidTwoColor(String section, LedColor color1, LedColor color2) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int hue1 = color1.hues();
        final int val1 = applyBrightness(color1.value());
        final int hue2 = color2.hues();
        final int val2 = applyBrightness(color2.value());

        for (int i = s; i < e; i++) {
            if ((i - s) % 2 == 0) {
                ledBuffer.setHSV(i, hue1, 255, val1);
            } else {
                ledBuffer.setHSV(i, hue2, 255, val2);
            }
        }
    }

    public void colorTest(String section, double hue) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int value = brightnessLimit; // explicit controller brightness
        final int hueInt = (int) hue;
        for (int i = s; i < e; i++) {
            ledBuffer.setHSV(i, hueInt, 255, value);
        }
    }

    public void rainbow(String section, int speed) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int length = Math.max(1, e - s);
        for (int i = s; i < e; i++) {
            final int hue = (m_rainbowFirstPixelHue + (i - s) * 180 / length) % 180;
            ledBuffer.setHSV(i, hue, 255, brightnessLimit);
        }
        m_rainbowFirstPixelHue += speed;
        m_rainbowFirstPixelHue %= 180;
    }

    public void strobe(String section, LedColor color, double duration) {
        strobe(section, color, LedColor.BLACK, duration);
    }

    public void strobe(String section, LedColor color1, LedColor color2, double duration) {
        if (!strobeTimer.advanceIfElapsed(duration / 2))
            return;

        if (!on) {
            solid(section, color1);
            on = true;
        } else {
            solid(section, color2);
            on = false;
        }
    }

    public void fade(String section, LedColor color1, LedColor color2, int cycleLength, double duration) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);

        double x = (1 - ((System.currentTimeMillis() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = s; i < e; i++) {
            x += xDiffPerLed;
            double ratio = (Math.pow(Math.sin(x), fadeExponent) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
                ratio = 0.5;
            }

            int outputColor = (int) Math.round((color1.hues() * (1 - ratio)) + (color2.hues() * ratio));
            ledBuffer.setHSV(i, outputColor, 255, applyBrightness(color1.value()));
        }
    }

    public void wave(String section, LedColor color, int speed) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int colorValue = applyBrightness(color.value());
        final int range = Math.max(1, e - s);
        for (int i = s; i < e; i++) {
            final int value = (m_waveValue + (i * colorValue / range)) % colorValue;
            ledBuffer.setHSV(i, color.hues(), 255, value);
        }
        m_waveValue += speed;
        m_waveValue %= 180;
    }

    public void breath(String section, LedColor color1, double duration) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);

        double x = ((System.currentTimeMillis() % duration) / duration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        for (int i = s; i < e; i++) {
            final int value = (int) (((applyBrightness(color1.value())) * (1 - ratio)) + (0 * ratio));
            ledBuffer.setHSV(i, color1.hues(), 255, value);
        }
    }

    public void fill(String section, LedColor color, int increment, double duration, boolean inverse) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int blackV = 0;
        for (int i = s; i < e; i++) {
            if (!inverse) {
                if (i <= s + m_range) {
                    ledBuffer.setHSV(i, color.hues(), 255, applyBrightness(color.value()));
                } else {
                    ledBuffer.setHSV(i, LedColor.BLACK.hues(), 255, blackV);
                }
            } else {
                if (i >= e - m_range) {
                    ledBuffer.setHSV(i, color.hues(), 255, applyBrightness(color.value()));
                } else {
                    ledBuffer.setHSV(i, LedColor.BLACK.hues(), 255, blackV);
                }
            }
        }

        double speed = increment * (duration / Math.max(1, (e - s)));
        if (!fillTimer.advanceIfElapsed(speed))
            return;

        m_range += increment;
        m_range %= Math.max(1, (e - s));
    }

    public void carnival(String section, LedColor color1, LedColor color2, int length, double speed) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        final int hue1 = color1.hues();
        final int val1 = applyBrightness(color1.value());
        final int hue2 = color2.hues();
        final int val2 = applyBrightness(color2.value());

        for (int i = s; i < e; i++) {
            if (starter) {
                if ((i - s) % 2 == 0) {
                    ledBuffer.setHSV(i, hue1, 255, val1);
                } else {
                    ledBuffer.setHSV(i, hue2, 255, val2);
                }
            } else {
                if ((i - s) % 2 == 0) {
                    ledBuffer.setHSV(i, hue2, 255, val2);
                } else {
                    ledBuffer.setHSV(i, hue1, 255, val1);
                }
            }
        }

        if (!carnivalTimer.advanceIfElapsed(speed))
            return;

        starter = !starter;
    }

    public void zip(String section, LedColor color, int length, int increment, double duration, boolean inverse) {
        final int s = LedSectionConfig.getSectionStart(section);
        final int e = LedSectionConfig.getSectionEnd(section);
        if (!inverse) {
            start = s + zipIncrease;
            end = start + length;
        } else {
            start = e - zipIncrease;
            end = start - length;
        }

        final int blackV = applyBrightness(LedColor.BLACK.value());
        final int colorV = applyBrightness(color.value());

        for (int i = s; i < e; i++) {
            if (!inverse) {
                if (i > start && i <= end) {
                    ledBuffer.setHSV(i, color.hues(), 255, colorV);
                } else {
                    ledBuffer.setHSV(i, LedColor.BLACK.hues(), 255, blackV);
                }
            } else {
                if (i < start && i >= end) {
                    ledBuffer.setHSV(i, color.hues(), 255, colorV);
                } else {
                    ledBuffer.setHSV(i, LedColor.BLACK.hues(), 255, blackV);
                }
            }
        }

        double speed = increment * (duration / Math.max(1, (e - s)));
        if (!zipTimer.advanceIfElapsed(speed))
            return;

        zipIncrease += increment;
        zipIncrease %= Math.max(1, (e - s) - length);
    }
}