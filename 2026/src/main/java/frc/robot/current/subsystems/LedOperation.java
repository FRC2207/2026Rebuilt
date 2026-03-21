package frc.robot.current.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.leds.LedColor;
import frc.robot.lib.leds.LedController;

public class LedOperation extends SubsystemBase {
  public static final LedController leds = new LedController(240, 3, .75);
  private LedColor color;
  private final SendableChooser<Runnable> m_chooser = new SendableChooser<>();
  private final SendableChooser<LedColor> m_color = new SendableChooser<>();

  // Constants regarding automatic LED states
  public ShuffleboardTab tab = Shuffleboard.getTab("Robot");
  public GenericEntry hueValue = tab.add("Hue Value", 0).getEntry();
  
  public boolean automaticLED = false;

  public boolean isPathing = false;
  public boolean isShooting = false;
  public boolean isIntaking = false;
  public boolean isClimbing = false;
  public boolean isInRange = false;

  public LedOperation() {
    leds.addSection("full", 0, 240);
    leds.addSection("right", 32, 80);
    leds.addSection("front", 81, 145);
    leds.addSection("left", 146, 191);
    leds.addSection("mechanismFrame", 32, 191);
    leds.addSection("underglow1", 0, 32);
    leds.addSection("underglow", 191, 238);

    m_chooser.setDefaultOption("Solid", () -> leds.solid("mechanismFrame", color));
    m_chooser.addOption("Two Color Solid", () -> leds.solidTwoColor("mechanismFrame", LedColor.TURQUOISE, LedColor.PEACH));
    m_chooser.addOption("Solid Black", () -> leds.solid("mechanismFrame", LedColor.BLACK));
    m_chooser.addOption("Rainbow", () -> leds.rainbow("mechanismFrame", 3));
    m_chooser.addOption("Fade Blue and Green", () -> leds.fade("mechanismFrame", LedColor.GREEN, LedColor.BLUE, 1, 3));
    m_chooser.addOption("Breath", () -> leds.breath("mechanismFrame", color, 3));
    m_chooser.addOption("Strobe", () -> leds.strobe("mechanismFrame", color, 1));
    m_chooser.addOption("Carnival", () -> leds.carnival("mechanismFrame", LedColor.EASTER_GREEN, LedColor.EASTER_PURPLE, 2, 4));
    m_chooser.addOption("Fill", () -> leds.fill("mechanismFrame", color, 1, 2, true));
    m_chooser.addOption("Zip", () -> leds.zip("mechanismFrame", color, 10, 1, 2, true));
    m_chooser.addOption("Wave", () -> leds.wave("mechanismFrame", color, 3));
    m_chooser.addOption("Color testing", () -> leds.colorTest("mechanismFrame", hueValue.getDouble(0)));

    m_color.addOption("Red", LedColor.RED);
    m_color.addOption("Red-Orange", LedColor.RED_ORANGE);
    m_color.setDefaultOption("Orange", LedColor.ORANGE);
    m_color.addOption("Gold", LedColor.GOLD);
    m_color.addOption("Yellow", LedColor.YELLOW);
    m_color.addOption("Yellow-Green", LedColor.YELLOW_GREEN);
    m_color.addOption("Lime", LedColor.LIME);
    m_color.addOption("Green", LedColor.GREEN);
    m_color.addOption("Aqua", LedColor.AQUA);
    m_color.addOption("Sea Blue", LedColor.SEA_BLUE);
    m_color.addOption("Light Blue", LedColor.LIGHT_BLUE);
    m_color.addOption("Sky Blue", LedColor.SKY_BLUE);
    m_color.addOption("Blue", LedColor.BLUE);
    m_color.addOption("Cornflower", LedColor.CORNFLOWER);
    m_color.addOption("Indigo", LedColor.INDIGO);
    m_color.addOption("Light Purple", LedColor.LIGHT_PURPLE);
    m_color.addOption("purple", LedColor.PURPLE);
    m_color.addOption("Light Pink", LedColor.LIGHT_PINK);
    m_color.addOption("Rose", LedColor.ROSE);
    m_color.addOption("Magenta", LedColor.MAGENTA);
    m_color.addOption("Brown", LedColor.BROWN);
    m_color.addOption("White", LedColor.WHITE);

    SmartDashboard.putData("Manual LED", m_chooser);
    SmartDashboard.putData("LEDColor", m_color);
  }

  @Override
  public void periodic() {
    color = m_color.getSelected();
    automaticLED = false;

    robotStatus();

    leds.updateLeds();
  }

  public void robotStatus() {
    if (DriverStation.isEStopped()) {
      leds.strobe("full", LedColor.RED, 1);
    } else if (DriverStation.isAutonomousEnabled()) {
      leds.rainbow("front", 4);
    } else if (DriverStation.isTeleopEnabled()) {
      if (automaticLED) {
        updateState();
      } else {
        manualState();
      }
    } else {
      if (DriverStation.isDSAttached()) {
        leds.fill("front", LedColor.ORANGE, 2, 2, false);
      } else if (DriverStation.isFMSAttached()) {
        leds.carnival("front", LedColor.ORANGE, LedColor.WHITE, 2, 1);
      } else {
        leds.breath("front", LedColor.ORANGE, 2);
      }
    }

    Optional<Alliance> ally = DriverStation.getAlliance();
    
      if (ally.get() == Alliance.Blue) {
        leds.solid("underglow", LedColor.BLUE);
        leds.solid("underglow1", LedColor.BLUE);
      } else {
        leds.solid("underglow", LedColor.RED);
        leds.solid("underglow1", LedColor.RED);
      }
    
  }

  /* Method to set the LEDs automatically depending on the robots state */
  public void updateState() {
      if (isPathing) {
        leds.rainbow("mechanismFrame", 3);
      } else {
        leds.solid("mechanismFrame", LedColor.ORANGE);
      }
      
      if (isShooting) {
        leds.strobe("intakeSide", LedColor.YELLOW, 1);
      } 
      
      else if (isIntaking) {
        leds.solidTwoColor("mechanismFrame", LedColor.GREEN, LedColor.BLACK);
      } else if (!isInRange) {
        leds.strobe("mechanismFrame", LedColor.RED_ORANGE, 1);
      } else if (isInRange) {
        leds.breath("mechanismFrame", LedColor.GREEN, 3);
      } else {
        leds.solid("mechanismFrame", LedColor.ORANGE);
      }

      if (isClimbing) {
        leds.fill("climbers", LedColor.PURPLE, 2, 2, false);
      }
  }

  /* Method to set the LEDs to different states during the match */
  public void manualState() {
    m_chooser.getSelected().run();
  }
}
