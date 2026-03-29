package frc.robot.current.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.PivotConstants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorControllerIO;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Pivot extends SubsystemBase {
  private MotorController pivotMotor;

  private final int pivotMotorID = Constants.PivotConstants.pivotID;
  private EncoderType encoderType = EncoderType.EXTERNAL_ABSOLUTE;

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
        .kG(PivotConstants.kG)
        .kCos(PivotConstants.kCos); // Acceleration gain (volts per RPM/s)

    switch (Constants.currentMode) {
      case REAL:
        pivotMotor = new MotorController(
            new MotorIOSpark(pivotMotorID, pivotConfig, SparkType.SparkMax, MotorModel.NeoV1, encoderType), "Pivot");
        break;
      case SIM:
        pivotMotor = new MotorController(new MotorIOSim(MotorModelSim.NeoV1, ControlType.Position, PivotConstants.kSim_P,
            PivotConstants.kSim_I, PivotConstants.kSim_D, 0.0, 0.0, 0.3, 1), "Pivot");
        break;
      default:
        // Blank IO for REPLAY
        pivotMotor = new MotorController(
            new MotorControllerIO() {}, "Pivot");
        break;
    }
  }

  public void periodic() {
    pivotMotor.updateInputs();
    Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Pivot/Setpoint", pivotMotor.getSetpoint()));
    Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Pivot/IsUp", isUp));

    if (pivotMotor.getPositionRotations() >= .2) {
      isUp = true;
    } else {
      isUp = false;
    }

    Logger.runEveryN(2, (Runnable) () -> Logger.recordOutput("Pivot/ComponentPose", new Pose3d[] {new Pose3d(0.182, 0.13, 0.2, new Rotation3d(0, -pivotMotor.getPositionRadians(), 0))}));
  }

  public void initialization() {
    setPivotPosition(pivotMotor.getPositionRotations());
  }

  public void setPivotPosition(double setpoint) {
    pivotMotor.setPositionRotations(setpoint);
  }

  public Command gotoStoredPos() {
    return Commands.runOnce(() -> {
      pivotMotor.setPositionRotations(Constants.PivotConstants.storedRotations);
    }, this);
  }

  public Command gotoCollectionPos() {
    return Commands.sequence(
      Commands.runOnce(() -> {
        pivotMotor.setPositionRotations(Constants.PivotConstants.intermediateRotations);
      }, this),
      Commands.waitSeconds(2.0),
      Commands.runOnce(() -> {
        pivotMotor.setPositionRotations(Constants.PivotConstants.collectionRotations);
      }, this)
    );
  }
}