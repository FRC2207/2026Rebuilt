package frc.robot.current.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.PivotConstants;
import frc.robot.lib.motors.positionController.PositionController;
import frc.robot.lib.motors.positionController.PositionIOSparkMax;

public class Pivot extends SubsystemBase {
  private PositionController pivotMotor;

  private final int pivotMotorID = Constants.PivotConstants.pivotID;
  private final String robotType = Constants.robot;

  public Boolean isUp = false;

  public Pivot() {

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(false);
    pivotConfig.smartCurrentLimit(30);
    pivotConfig.absoluteEncoder.inverted(true);
    pivotConfig.absoluteEncoder.zeroCentered(true);

    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    pivotConfig.closedLoop
        .p(PivotConstants.kP)
        .i(PivotConstants.kI)
        .d(PivotConstants.kD).feedForward // Set Feedforward gains for the velocity controller
        .kS(PivotConstants.kS) // Static gain (volts)
        .kV(PivotConstants.kV) // Velocity gain (volts per RPM)
        .kA(PivotConstants.kA)
        .kG(PivotConstants.kG); // Acceleration gain (volts per RPM/s)

    switch (robotType) {
      case "Real":
        pivotMotor = new PositionController(new PositionIOSparkMax(pivotMotorID, pivotConfig, 0.0), "Pivot");
        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        pivotMotor = new PositionController(new PositionIOSparkMax(pivotMotorID, pivotConfig, 0.0), "Pivot");
        break;
    }
  }

  public void periodic() {
    pivotMotor.updateInputs();
    Logger.recordOutput("pivotSetpoint", pivotMotor.getMotorSetpoint());
    SmartDashboard.putBoolean("IsUp", isUp);

    if (pivotMotor.getAngle() >= .2) {
      isUp = true;
    } else {
      isUp = false;
    }
  }

  public void initialization() {
    setPivotPosition(pivotMotor.getAngle());
  }

  public void setPivotPosition(double setpoint) {
    pivotMotor.setMotorPosition(setpoint);
  }

  public Command gotoStoredPos() {
    return Commands.run(() -> {
      pivotMotor.setMotorPosition(Constants.PivotConstants.storedRotations);
    }, this);
  }

  public Command gotoCollectionPos() {
    return Commands.run(() -> {
      pivotMotor.setMotorPosition(Constants.PivotConstants.collectionRotations);
    }, this);
  }

  // DO NOT USE
  public Command rotateUp() {
    // THE value HAS TO BE EXTREMLY SMALL AS IT IS IN ROTATIONS AND NOT ANGLES
    return Commands.runOnce(() -> {
      isUp = true;
      pivotMotor.setMotorPositionDegrees(pivotMotor.getMotorSetpointDegrees() + 1);
    }, this);
  }

  // DO NOT USE
  public Command rotateDown() {
    // THE - value HAS TO BE EXTREMLY SMALL AS IT IS IN ROTATIONS AND NOT ANGLES
    return Commands.runOnce(() -> {
      isUp = false;
      pivotMotor.setMotorPositionDegrees(pivotMotor.getMotorSetpointDegrees() - 1);
    }, this);
  }
}