package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;


public class Intake extends SubsystemBase {
  private MotorController intakeMotor;

  private final int intakeMotorID = Constants.IntakeConstants.intakeID;
  
  public Intake(String robotType, Drive drive) {
    // TODO: change table and entry keys
    // table = NetworkTableInstance.getDefault().getTable("default");
    // visionTarget = table.getEntry("default");

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true);
    rightConfig.smartCurrentLimit(30);

    switch (robotType) {
      case "Real":
        intakeMotor = new MotorController(new MotorIOSparkMax(intakeMotorID, rightConfig, 35), "Intake", "1");

        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        intakeMotor = new MotorController(new MotorIOSparkMax(intakeMotorID, rightConfig, 30), "Intake", "1");

        break;
    }
  }

  public void periodic() {
    intakeMotor.updateInputs();
  }


  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public Command launch() {
    double percent = 20;

    return Commands.sequence(
        runOnce(() -> {
          intakeMotor.setPercent(percent);
        }),
        Commands.waitSeconds(.5),
        runOnce(() -> {
          intakeMotor.setPercent(0);
        }));
  }

  public Command intake() {
    return Commands.run(() -> {
      intakeMotor.setPercent(Constants.IntakeConstants.intakeSpeed);
    },
        this);
  }

  public Command stop() {
    return Commands.run(() -> {
      intakeMotor.setVoltage(0);
    },
        this);
  }
}