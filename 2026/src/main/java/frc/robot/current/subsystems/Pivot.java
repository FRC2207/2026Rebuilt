package frc.robot.current.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.PivotConstants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Pivot extends SubsystemBase {
  private MotorController pivotMotor;

  private final int pivotMotorID = Constants.PivotConstants.pivotID;

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

    switch (Constants.currentMode) {
      case REAL:
        pivotMotor = new MotorController(
            new MotorIOSpark(pivotMotorID, pivotConfig, SparkType.SparkMax, MotorModel.NeoV1), "Pivot");
        break;
      case SIM:
        pivotMotor = new MotorController(new MotorIOSim(MotorModelSim.NeoV1, ControlType.Position, PivotConstants.kSim_P,
            PivotConstants.kSim_I, PivotConstants.kSim_D, 0.0, 0.0, 0.3, 1), "Pivot");
        break;
      default:
        pivotMotor = new MotorController(
            new MotorIOSpark(pivotMotorID, pivotConfig, SparkType.SparkMax, MotorModel.NeoV1), "Pivot");
        break;
    }
  }

  public void periodic() {
    pivotMotor.updateInputs();
    Logger.recordOutput("Pivot/Setpoint", pivotMotor.getSetpointRotations());
    Logger.recordOutput("Pivot/IsUp", isUp);

    if (pivotMotor.getPositionRotations() >= .2) {
      isUp = true;
    } else {
      isUp = false;
    }
  }

  public void initialization() {
    setPivotPosition(pivotMotor.getPositionRotations());
  }

  public void setPivotPosition(double setpoint) {
    pivotMotor.setPositionRotations(setpoint);
  }

  public Command gotoStoredPos() {
    return Commands.run(() -> {
      pivotMotor.setPositionRotations(Constants.PivotConstants.storedRotations);
    }, this);
  }

  public Command gotoCollectionPos() {
    return Commands.run(() -> {
      pivotMotor.setPositionRotations(Constants.PivotConstants.collectionRotations);
    }, this);
  }
}