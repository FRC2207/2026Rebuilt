package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;

public class ExampleFlyWheel extends SubsystemBase{
    private MotorController rightMotor;
    private MotorController leftMotor;

    private final int rightMotorID = 68;
    private final int leftMotorID = 69;

    private boolean currentTriggered = false;
    private double currentTriggerTime = Double.MAX_VALUE;

    public boolean hasAlgae = false;

    public ExampleFlyWheel(String robotType) {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.inverted(true);
        leftConfig.follow(rightMotorID, true);
        leftConfig.smartCurrentLimit(30);
        rightConfig.smartCurrentLimit(30
        );

        switch (robotType) {
            case "Real":
                rightMotor = new MotorController(new MotorIOSparkMax(rightMotorID, rightConfig, 35), "Algae", "1");
                leftMotor = new MotorController(new MotorIOSparkMax(leftMotorID, leftConfig, 35), "Algae", "2");

                break;
            case "SIM":
                // Just don't use sim.

                break;
            default:
                rightMotor = new MotorController(new MotorIOSparkMax(rightMotorID, rightConfig, 30), "Algae", "1");
                leftMotor = new MotorController(new MotorIOSparkMax(leftMotorID, leftConfig, 30),"Algae", "2");
                
                break;
        }
    }

    public void periodic() {
        rightMotor.updateInputs();
        leftMotor.updateInputs();
    }

    public void setVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }
    
    public Command launch() {
      double percent = 20;
    
        return Commands.sequence(
          runOnce(() -> {
            rightMotor.setPercent(percent);
            hasAlgae = false;
          }), 
          Commands.waitSeconds(.5), 
          runOnce(() -> {
            rightMotor.setPercent(0);
          })
        );
    }

    public Command stop() {
        return Commands.run(() -> {
            rightMotor.setVoltage(0);
        }, 
        this);
    }

    /** Returns a command that runs the intake continuously until the limit switches are pressed then stops. */
  public Command continuousIntakeCommand() {
    return Commands.sequence(
      runOnce(() -> {
        rightMotor.setPercent(-.4);
      }),
      Commands.waitUntil(() -> rightMotor.hasCurrentReached() == true || aboveAmpThreshold() == true).withTimeout(10),
      runOnce(() -> {
        if (rightMotor.hasCurrentReached()) {
          hasAlgae = true;
        } else {
          hasAlgae = false;
        }
      })
    );
  }

  public Command defualtCommand() {
    return Commands.run(() -> {
      if (hasAlgae) {
        rightMotor.setPercent(-.25);
        if (rightMotor.getCurrent() <= 30) {
          hasAlgae = false;
        }
      }
    },
    this);
  }

  public boolean aboveAmpThreshold() {
    if (!currentTriggered && rightMotor.hasCurrentReached()) {
      currentTriggered = true;
      currentTriggerTime = Timer.getFPGATimestamp();
    }
    
    if (currentTriggered && rightMotor.hasCurrentReached()) {
      currentTriggered = false;
    }

    if (currentTriggered && Timer.getFPGATimestamp() - currentTriggerTime > 0.4) {
      return true;
    }
    return false;
  }
}
