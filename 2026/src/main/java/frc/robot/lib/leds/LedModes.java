package frc.robot.lib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LedModes {
    public AddressableLED m_Led;
    private AddressableLEDBuffer ledBuffer;

    private final Timer strobeTimer = new Timer();
    private final Timer zipTimer = new Timer();
    private final Timer fillTimer = new Timer();
    private final Timer carnivalTimer = new Timer();

    private int brightnessLimit = 100;
    private int m_rainbowFirstPixelHue;
    private int m_waveValue;
    private int m_range;
    private int zipIncrease;
    private int start;
    private int end;
    private static final double fadeExponent = 0.4;
    public boolean on = true;
    public boolean starter = true;

    /**
     * Configures the LED's length, PWM port, and brightness
     * 
     * @param length     is the total number of LEDs you would like to operate
     * @param port       is the PWM port the LEDs are plugged into
     * @param brightness is how bright you would like you LEDs. Value between 0 and
     *                   1
     */
    public LedModes(int length, int port, int brightness) {
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

    protected void setData() {
        m_Led.setData(ledBuffer);
    }

    public int getLedBrightness() {
        return brightnessLimit;
    }

    /**
     * Sets the strip to a solid. Currently only a single color
     * 
     * @param section is the range you want to add the effect
     * @param color   is the color you want the LEDs to be
     */
    public void solid(String section, LedColor color) {
        for (var i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            final var hue = color.hues();
            final var value = color.value();
            // Sets the specified LED to the HSV values for the preferred color
            ledBuffer.setHSV(i, hue, 255, value);
        }
    }

    /**
     * Sets the strip to a solid. Currently only a single color
     * 
     * @param section is the range you want to add the effect
     * @param color1  is the color you want the LEDs to be
     * @param color2  is the second color you want to display
     */
    public void solidTwoColor(String section, LedColor color1, LedColor color2) {

        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            if (i % 2 == 0) {
                final var hue = color1.hues();
                final var value = color1.value();
                // Sets the specified LED to the HSV values for the preferred color
                ledBuffer.setHSV(i, hue, 255, value);
            } else {
                final var hue = color2.hues();
                final var value = color2.value();
                // Sets the specified LED to the HSV values for the preferred color
                ledBuffer.setHSV(i, hue, 255, value);
            }
        }
    }

    /**
     * Sets the strip to a solid color based on the hue given instead of a preset
     * color
     * 
     * @param section is the range you want to add the effect
     * @param hue     is the hue you want to send to the light
     */
    public void colorTest(String section, double hue) {
        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            final var value = brightnessLimit;
            // Sets the specified LED to the HSV values for the preferred color
            ledBuffer.setHSV(i, (int) hue, 255, value);
        }
    }

    /**
     * Sets the strip to a rainbow pattern that moves along the light strand
     * 
     * @param section is the range you want to add the effect
     * @param speed   is the speed you want the rainbow to run down the strip - 5 is
     *                recommended for long distance, 3 is recommended for short
     *                distances
     */
    public void rainbow(String section, int speed) {
        // For designate range
        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue
                    + (i * 180 / (LedSectionConfig.getSectionEnd(section) - LedSectionConfig.getSectionStart(section)))
                            % 180);
            // Set the value
            ledBuffer.setHSV(i, hue, 255, brightnessLimit);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += speed;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    /**
     * Fades between the colors designated - on the hue scale
     * 
     * @param section     is the range you want to add the effect
     * @param color1      is the base color
     * @param color2      is the color to fade in and out of
     * @param cycleLength is the frequency you fade the color by the pixels
     * @param duration    is the time it takes to cycle back to the original color
     */
    public void fade(String section, LedColor color1, LedColor color2, int cycleLength, double duration) {
        double x = (1 - ((System.currentTimeMillis() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        final var value = color1.value();
        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            x += xDiffPerLed;
            if (i >= LedSectionConfig.getSectionStart(section)) {
                double ratio = (Math.pow(Math.sin(x), fadeExponent) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(x + Math.PI), fadeExponent) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) {
                    ratio = 0.5;
                }

                int outputColor = (int) Math.round((color1.hues() * (1 - ratio)) + (color2.hues() * ratio));
                ledBuffer.setHSV(i, outputColor, 255, value);
            }
        }
    }

    /**
     * Moves an area of color through the strip as a soft fade rather than an adrupt
     * end like {@link LedModes#zip}
     * 
     * @param section is the range you want to add the effect
     * @param speed   is the speed you want the lit section to run down the strip -
     *                5 is
     *                recommended for long distance, 3 is recommended for short
     *                distances
     */
    public void wave(String section, LedColor color, int speed) {
        // For designate range
        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var value = (m_waveValue + (i * color.value() / LedSectionConfig.getSectionEnd(section)))
                    % color.value();
            // Set the value
            ledBuffer.setHSV(i, color.hues(), 255, value);
        }
        // Increase by to make the rainbow "move"
        m_waveValue += speed;
        // Check bounds
        m_waveValue %= 180;
    }

    /**
     * Pulses the lights to give them a breathing effect.
     * 
     * @param section  is the range you want to add the effect
     * @param color1   is the color that with breath
     * @param duration is the time it takes to go through 1 cycle
     */
    public void breath(String section, LedColor color1, double duration) {
        double x = ((System.currentTimeMillis() % duration) / duration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        LedColor color2 = LedColor.BLACK;

        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            final var value = (color1.value() * (1 - ratio)) + (color2.value() * ratio);
            ledBuffer.setHSV(i, color1.hues(), 255, (int) value);
        }
    }

    /**
     * Sets the LEDS to a striped pattern of infinite colors of users choosing
     * 
     * @param colors       is the list of colors you want to add
     * @param stripeLength is how many LEDS you want each color to possess
     * @param duration     is how quickly you want the LEDS to move down the strip
     */
    /*
     * private void stripes(List<Color> colors, int stripeLength, double duration) {
     * int offset =
     * (int) (System.currentTimeMillis() % duration / duration * stripeLength *
     * colors.size());
     * 
     * for (int i = 0; i < length; i++) {
     * int colorIndex =
     * (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) %
     * colors.size();
     * colorIndex = colors.size() - 1 - colorIndex;
     * ledBuffer.setHSV(i, colors.get(colorIndex), 255, brightnessLimit);
     * }
     * }
     */

    /**
     * Flashes the LEDs on and off at the designated speed
     * 
     * @param section  is the range you want to add the effect
     * @param color    is the color that will flash
     * @param duration is the time between each flash
     */
    public void strobe(String section, LedColor color, double duration) {
        strobe(section, color, LedColor.BLACK, duration);
    }

    /**
     * Flashes the LEDs between two colors at the designated speed
     * 
     * @param section  is the range you want to add the effect
     * @param color1   is the base color
     * @param color2   is the secondary color
     * @param duration is the time between each flash
     */
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

    /**
     * Fills the section of LEDs with the color incrementally
     * 
     * @param section   is the range you want to add the effect
     * @param color     is the color you want to fill
     * @param increment is how many LEDs you want to fill per cycle
     * @param duration  is the length of time it takes to go through the whole
     *                  section. Currently restricted at greater than 2
     * @param inverse   is the direction you would like to go - false starts at the
     *                  source, true goes towards the source
     */
    public void fill(String section, LedColor color, int increment, double duration, boolean inverse) {
        LedColor color2 = LedColor.BLACK;
        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            if (!inverse) {
                if (i <= LedSectionConfig.getSectionStart(section) + m_range) {
                    ledBuffer.setHSV(i, color.hues(), 255, color.value());
                } else {
                    ledBuffer.setHSV(i, color2.hues(), 255, color2.value());
                }
            } else {
                if (i >= LedSectionConfig.getSectionEnd(section) - m_range) {
                    ledBuffer.setHSV(i, color.hues(), 255, color.value());
                } else {
                    ledBuffer.setHSV(i, color2.hues(), 255, color2.value());
                }
            }
        }
        // Only pass if the time calculated has passed
        double speed = increment
                * (duration / (LedSectionConfig.getSectionEnd(section) - LedSectionConfig.getSectionStart(section)));
        if (!fillTimer.advanceIfElapsed(speed))
            return;

        // increase to fill the strip
        m_range += increment;
        // check bounds
        m_range %= LedSectionConfig.getSectionEnd(section) - LedSectionConfig.getSectionStart(section);
    }

    /**
     * 2 Color mode that appears to move along the section
     * 
     * @param section is the range you want to add the effect
     * @param color1  is the first color you would like to display
     * @param color2  is the second color you would ike to display
     * @param length  is how many LEDs each color represents per cycle
     * @param speed   is how fast you want the lights to move along the section
     *
     */
    public void carnival(String section, LedColor color1, LedColor color2, int length, double speed) {
        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            if (starter) {
                if (i % 2 == 0) {
                    final var hue = color1.hues();
                    final var value = color1.value();
                    // Sets the specified LED to the HSV values for the preferred color
                    ledBuffer.setHSV(i, hue, 255, value);
                } else {
                    final var hue = color2.hues();
                    final var value = color2.value();
                    // Sets the specified LED to the HSV values for the preferred color
                    ledBuffer.setHSV(i, hue, 255, value);
                }
            } else {
                if (i % 2 == 0) {
                    final var hue = color2.hues();
                    final var value = color2.value();
                    // Sets the specified LED to the HSV values for the preferred color
                    ledBuffer.setHSV(i, hue, 255, value);
                } else {
                    final var hue = color1.hues();
                    final var value = color1.value();
                    // Sets the specified LED to the HSV values for the preferred color
                    ledBuffer.setHSV(i, hue, 255, value);
                }
            }
        }

        if (!carnivalTimer.advanceIfElapsed(speed))
            return;

        if (starter) {
            starter = false;
        } else {
            starter = true;
        }
    }

    /**
     * Moves a smaller section of leds through the entire section
     * 
     * @param section   is the range you want to add the effect
     * @param color     is the color you want to display
     * @param length    is the length of the strip of activated lights
     * @param increment is how many LEDs you want to add per cycle
     * @param duration  is the length of time it takes to go through the entire
     *                  section. Currently restricted at greater than 2
     * @param inverse   is the direction you would like to go - false starts at the
     *                  source, true goes towards the source
     */
    public void zip(String section, LedColor color, int length, int increment, double duration, boolean inverse) {
        LedColor color2 = LedColor.BLACK;
        if (!inverse) {
            start = LedSectionConfig.getSectionStart(section) + zipIncrease;
            end = start + length;
        } else {
            start = LedSectionConfig.getSectionEnd(section) - zipIncrease;
            end = start - length;
        }

        for (int i = LedSectionConfig.getSectionStart(section); i < LedSectionConfig.getSectionEnd(section); i++) {
            if (!inverse) {
                if (i > start && i <= end) {
                    ledBuffer.setHSV(i, color.hues(), 255, color.value());
                } else {
                    ledBuffer.setHSV(i, color2.hues(), 255, color2.value());
                }
            } else {
                if (i < start && i >= end) {
                    ledBuffer.setHSV(i, color.hues(), 255, color.value());
                } else {
                    ledBuffer.setHSV(i, color2.hues(), 255, color2.value());
                }
            }
        }

        // Only pass if the time calculated has passed
        double speed = increment
                * (duration / (LedSectionConfig.getSectionEnd(section) - LedSectionConfig.getSectionStart(section)));
        if (!zipTimer.advanceIfElapsed(speed))
            return;

        // increase to move the strip
        zipIncrease += increment;
        // check bounds
        zipIncrease %= LedSectionConfig.getSectionEnd(section) - LedSectionConfig.getSectionStart(section) - length;
    }
}
