package frc.robot.lib.leds;

import edu.wpi.first.math.MathUtil;

/**
 * This is the controller for all needs regarding LEDs. Once you create your LED
 * object, you must create sections using {@link LedController#addSection} in
 * your constructor. Finally, include {@link LedController#updateLeds} in your periodic
 */
public class LedController {
    /** The total length (in LEDs) you would like to operate. */
    private final int ledLength;

    /** The PWM port in which the leds are plugged into the RoboRio. */
    private final int ledPort;

    /**
     * The brightness level of your LEDs (0..255).
     */
    private int ledBrightness = 255;

    /** Composition: effects implementation that owns the buffer / timers */
    private final LedEffects effects;

    /**
     * Configures the LED's length, PWM port, and brightness
     * 
     * @param length     total number of LEDs
     * @param port       PWM port the LEDs are plugged into
     * @param brightness brightness between 0 and 1
     */
    public LedController(int length, int port, double brightness) {
          int b255 = (int) (MathUtil.clamp(brightness, 0.0, 1.0) * 255.0);
        this.ledLength = length;
        this.ledPort = port;
        this.ledBrightness = b255;

        // create effects instance which owns the AddressableLEDBuffer and timers
        this.effects = new LedEffects(length, port, b255);
    }

    /**
     * This is how you tell the code which LEDs to light. You can have as many
     * sections as you would like, and they may overlap. Call once during initialization (e.g., in robot constructor).
     * 
     * @param title is the name of the section you are controlling. It can be
     *              whatever you desire, but must be consistent through your code
     *              per section
     * @param start is the LED ID that the section starts at. Untested whether this
     *              needs to be smaller than the end
     * @param end   is the LED ID that the section ends at. Untested whether this
     *              needs to be greater than the start
     */
    public void addSection(String title, int start, int end) {
        LedSectionConfig.addSection(title, start, end);
    }

    /**
     * This is required to update the LEDs. If this is not included in your periodic, the LEDs will not update.
     */
    public void updateLeds() {
        effects.setData();
    }

    /** Get current brightness (0..255). */
    public int getLedBrightness() {
        return ledBrightness;
    }

    public int getLedLength() {
        return ledLength;
    }
    
    public int getLedPort() {
        return ledPort;
    }

    /** Set brightness (0..1). Updates internal effects brightness too. */
    public void setBrightness(double brightness) {
        int b255 = (int) (MathUtil.clamp(brightness, 0.0, 1.0) * 255.0);
        this.ledBrightness = b255;
        effects.setBrightness(b255);
    }

    /**
     * Sets the strip to a solid. Currently only a single color
     * 
     * @param section is the range you want to add the effect
     * @param color   is the color you want the LEDs to be
     */
    public void solid(String section, LedColor color) {
        effects.solid(section, color);
    }

    /**
     * Sets the strip to a solid. Currently only a single color
     * 
     * @param section is the range you want to add the effect
     * @param color1  is the color you want the LEDs to be
     * @param color2  is the second color you want to display
     */
    public void solidTwoColor(String section, LedColor color1, LedColor color2) {
        effects.solidTwoColor(section, color1, color2);
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
        effects.rainbow(section, speed);
    }

    /**
     * Pulses the lights to give them a breathing effect.
     * 
     * @param section  is the range you want to add the effect
     * @param color1   is the color that with breath
     * @param duration is the time it takes to go through 1 cycle
     */
    public void breath(String section, LedColor color, double duration) {
        effects.breath(section, color, duration);
    }

        /**
     * Flashes the LEDs on and off at the designated speed
     * 
     * @param section  is the range you want to add the effect
     * @param color    is the color that will flash
     * @param duration is the time between each flash
     */
    public void strobe(String section, LedColor color, double duration) {
        effects.strobe(section, color, duration);
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
        effects.strobe(section, color1, color2, duration);
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
        effects.fill(section, color, increment, duration, inverse);
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
        effects.carnival(section, color1, color2, length, speed);
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
        effects.zip(section, color, length, increment, duration, inverse);
    }

    /**
     * Sets the strip to a solid color based on the hue given instead of a preset
     * color
     * 
     * @param section is the range you want to add the effect
     * @param hue     is the hue you want to send to the light
     */
    public void colorTest(String section, double hue) {
        effects.colorTest(section, hue);
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
        effects.wave(section, color, speed);
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
        effects.fade(section, color1, color2, cycleLength, duration);
    }
}
