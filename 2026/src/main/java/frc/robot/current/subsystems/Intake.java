package frc.robot.current.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.IntakeConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Intake extends SubsystemBase {
  private MotorController intakeMotor;

  private final int intakeMotorId = Constants.IntakeConstants.intakeID;

  public Boolean isIntaking = false;

  public Intake(Drive drive) {

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.inverted(true);
    intakeConfig.smartCurrentLimit(30);
    intakeConfig.closedLoop
        .p(IntakeConstants.kP)
        .i(IntakeConstants.kI)
        .d(IntakeConstants.kD).feedForward // Set Feedforward gains for the velocity controller
        .kS(IntakeConstants.kS) // Static gain (volts)
        .kV(IntakeConstants.kV) // Velocity gain (volts per RPM)
        .kA(IntakeConstants.kA); // Acceleration gain (volts per RPM/s)

    switch (Constants.currentMode) {
      case REAL:
        intakeMotor = new MotorController(
            new MotorIOSpark(intakeMotorId, intakeConfig, SparkType.SparkFlex, MotorModel.Vortex, EncoderType.BUILTIN_RELATIVE), "Intake");
        break;
      case SIM:
        intakeMotor = new MotorController(new MotorIOSim(MotorModelSim.Vortex, ControlType.Velocity,
            IntakeConstants.kSim_P, IntakeConstants.kSim_I, IntakeConstants.kSim_D, IntakeConstants.kSim_G,
            IntakeConstants.kSim_V, IntakeConstants.kSim_MOI, IntakeConstants.kSim_GearReduction), "Intake");
        break;
      default:
        intakeMotor = new MotorController(new MotorIOSpark(intakeMotorId, intakeConfig, SparkType.SparkFlex, MotorModel.Vortex, EncoderType.BUILTIN_RELATIVE), "Intake");
        break;
    }
  }

  public void periodic() {
    intakeMotor.updateInputs();

    // NOTE: using getSetpointRotations() because their is no setpoint retrival for velocity control
    Logger.recordOutput("Intake/SetpointRPM", intakeMotor.getSetpoint());
  }

  public Command spit() {
    double percent = 2000;

    return Commands.sequence(
        runOnce(() -> {
          intakeMotor.setSpeedRPM(percent);
        }),
        Commands.waitSeconds(.5),
        runOnce(() -> {
          intakeMotor.setSpeedRPM(percent);
        }));
  }

  public Command intake() {
    return Commands.runOnce(() -> {
      isIntaking = true;
      intakeMotor.setSpeedRPM(IntakeConstants.intakeSpeed);
    }, this);
  }

  public Command stop() {
    return Commands.run(() -> {
      isIntaking = false;
      intakeMotor.setSpeedRPM(0);
      ;
    }, this);
  }
}