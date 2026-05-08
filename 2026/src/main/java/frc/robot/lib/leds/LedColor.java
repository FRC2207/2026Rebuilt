package frc.robot.lib.leds;

/**
 * This is a enum containing 37 LED colors in the HSV format.
 * Colors must be pulled from this file to be compatible with the White Bear
 * Lake LED Library.
 * 
 * If a color is not supported,
 * please use the colorTest() mode and comment on the GitHub repository with the
 * color recommendation (include web browser hue value)
 * Please keep in mind that these colors were tested with WS2812 light strands
 * and may show up differently on other light strands.
 * 
 * If you want to use a WPILib color multiply the hue value by 2
 * Hue values are divided by 2 because color pickers like Google are x/360, whereas WPILib is x/180
 * The value is set to the brightness of the LED, which is set in the LedController class.
 * If custom value is required add the value to the switch case in the value() method.
 */
public enum LedColor {
    LIGHT_RED(0, 192),
    SALMON(0, 125),
    RED(0, 125),
    RED_ORANGE(6, 125),
    PEACH(10, 125),
    ORANGE(10, 125),
    LIGHT_ORANGE(20, 125),
    GOLD(30, 125),
    LIGHT_YELLOW(50, 125),
    YELLOW(50, 125),
    SOFT_LIME(76, 125),
    YELLOW_GREEN(76, 125),
    EASTER_GREEN(90, 125),
    LIME(90, 125),
    LIGHT_GREEN(110, 125),
    GREEN(110, 125),
    AQUA(140, 125),
    SEA_GREEN(140, 125),
    SEA_BLUE(160, 125),
    LIGHT_BLUE(180, 125),
    TURQUOISE(180, 125),
    BABY_BLUE(200, 125),
    CORNFLOWER(200, 125),
    SKY_BLUE(215, 125),
    DARK_SKY_BLUE(240, 125),
    BLUE(240, 125),
    INDIGO(260, 125),
    LIGHT_PURPLE(260, 125),
    EASTER_PURPLE(276, 125),
    PURPLE(276, 125),
    BRIGHT_PURPLE(290, 125),
    LIGHT_PINK(336, 125),
    PINK(336, 125),
    ROSE(356, 125),
    MAGENTA(356, 125),
    BROWN(20, 70),
    WHITE(0, 0),
    BLACK(0, 0);

    private final int hue;
    private final int saturation;

    LedColor(int hue, int saturation) {
        this.hue = hue / 2;
        this.saturation = saturation;
    }

    public int hues() {
        return hue;
    }

    public int saturation() {
        return saturation;
    }

    /**
     * Returns the color's intrinsic brightness/value (0..255). Controller-level
     * brightness is applied by LedEffects/LedController when writing to the buffer.
     */
    public int value() {
        switch (this) {
            case BROWN:
                return 60;
            case BLACK:
                return 0;
            default:
                return 255;
        }
    }
}
