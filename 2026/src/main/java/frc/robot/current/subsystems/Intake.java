package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.IntakeConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkFlex;
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSparkMax;

public class Intake extends SubsystemBase {
  private MotorController intakeMotor;
  private PositionController pivotMotor;

  private final int intakeMotorID = Constants.IntakeConstants.motorID;
  private final int pivotMotorID = Constants.IntakeConstants.pivotMotorID;
  private final String robotType = Constants.robot;
  
  public Intake(Drive drive) {

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.inverted(true);
    intakeConfig.smartCurrentLimit(30);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(false);
    pivotConfig.smartCurrentLimit(30);

    pivotConfig.closedLoop
            .p(IntakeConstants.kP)
            .i(IntakeConstants.kI)
            .d(IntakeConstants.kD)
        .feedForward // Set Feedforward gains for the velocity controller
            .kS(IntakeConstants.kS) // Static gain (volts)
            .kV(IntakeConstants.kV) // Velocity gain (volts per RPM)
            .kA(IntakeConstants.kA); // Acceleration gain (volts per RPM/s)

    switch (robotType) {
      case "Real":
        intakeMotor = new MotorController(new MotorIOSparkFlex(intakeMotorID, intakeConfig, 35), "Intake", "1");
        pivotMotor = new PositionController(new PositionIOSparkMax(pivotMotorID, pivotConfig, 0.0),"Intake");
        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        intakeMotor = new MotorController(new MotorIOSparkFlex(intakeMotorID, intakeConfig, 30), "Intake", "1");
        pivotMotor = new PositionController(new PositionIOSparkMax(pivotMotorID, pivotConfig,0.0), "Intake");
        break;
    }
  }

  public void periodic() {
    intakeMotor.updateInputs();
    pivotMotor.updateInputs();
  }

  public void initialization(){
    setPivotPosition(pivotMotor.getAngle());
  }

  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public void setPivotPosition(double setpoint){
    pivotMotor.setMotorPosition(setpoint);
  }

  public Command spit() { 
    double percent = 0.2;

    return Commands.sequence(
        runOnce(() -> {
          intakeMotor.setPercent(percent);
        }),
        Commands.waitSeconds(.5),
        runOnce(() -> {
          intakeMotor.setPercent(0);
        }));
  }

  public Command gotoStoredPos() {
    return Commands.runOnce(() -> {
        pivotMotor.setMotorPosition(Constants.IntakeConstants.storedRotations);
    }, this);
  }

  public Command gotoCollectionPos() {
    return Commands.runOnce(() -> {
        pivotMotor.setMotorPosition(Constants.IntakeConstants.collectionRotations);
    }, this);
  }

  // DO NOT USE
  public Command rotateUp() {
    // THE value HAS TO BE EXTREMLY SMALL AS IT IS IN ROTATIONS AND NOT ANGLES
    return Commands.run(() -> {
      pivotMotor.setMotorPosition(pivotMotor.getMotorSetpoint() + 0.01);
    }, this);
  }

  // DO NOT USE
  public Command rotateDown() {
    // THE - value HAS TO BE EXTREMLY SMALL AS IT IS IN ROTATIONS AND NOT ANGLES
    return Commands.run(() -> {
      pivotMotor.setMotorPosition(pivotMotor.getMotorSetpoint() - 0.01);
    }, this);
  }

  public Command intake() {
    return Commands.runOnce(() -> {
      intakeMotor.setPercent(Constants.IntakeConstants.intakeSpeed);
    }, this);
  }

  public Command stop() {
    return Commands.run(() -> {
      intakeMotor.setVoltage(0);
    }, this);
  }
}