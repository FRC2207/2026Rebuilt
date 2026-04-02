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
import frc.robot.lib.motors.motorController.MotorControllerIO;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Intake extends SubsystemBase {
  private MotorController intakeMotorA;
  private MotorController intakeMotorB;

  private final int intakeMotorId = Constants.IntakeConstants.intakeID;
  private final int followerMotorId = Constants.IntakeConstants.followerID;

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

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.follow(intakeMotorId, true);

    switch (Constants.currentMode) {
      case REAL:
        intakeMotorA = new MotorController(
            new MotorIOSpark(intakeMotorId, intakeConfig, SparkType.SparkFlex, MotorModel.Vortex,
                EncoderType.BUILTIN_RELATIVE),
            "Intake/Leader" + intakeMotorId);
        intakeMotorB = new MotorController(
            new MotorIOSpark(followerMotorId, followerConfig, SparkType.SparkFlex, MotorModel.Vortex,
                EncoderType.BUILTIN_RELATIVE),
            "Intake/Follower" + followerMotorId);
        break;
      case SIM:
        intakeMotorA = new MotorController(new MotorIOSim(MotorModelSim.Vortex, ControlType.Velocity,
            IntakeConstants.kSim_P, IntakeConstants.kSim_I, IntakeConstants.kSim_D, IntakeConstants.kSim_S,
            IntakeConstants.kSim_V, IntakeConstants.kSim_MOI, IntakeConstants.kSim_GearReduction), "Intake");
        intakeMotorB = new MotorController(new MotorIOSim(MotorModelSim.Vortex, ControlType.Velocity,
            IntakeConstants.kSim_P, IntakeConstants.kSim_I, IntakeConstants.kSim_D, IntakeConstants.kSim_S,
            IntakeConstants.kSim_V, IntakeConstants.kSim_MOI, IntakeConstants.kSim_GearReduction), "IntakeA");
        break;
      default:
        // Blank IO for REPLAY
        intakeMotorA = new MotorController(new MotorControllerIO() {}, "IntakeA");
        intakeMotorB = new MotorController(new MotorControllerIO() {}, "IntakeB");

        break;
    }
  }

  public void periodic() {
    intakeMotorA.updateInputs();
    intakeMotorB.updateInputs();

    // NOTE: using getSetpointRotations() because their is no setpoint retrival for
    // velocity control
    Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Intake/SetpointRPM", intakeMotorA.getSetpoint()));
  }

  public Command spit() {
    double percent = -2000;

    return Commands.sequence(
        runOnce(() -> {
          intakeMotorA.setSpeedRPM(percent);
        }),
        Commands.waitSeconds(.5),
        runOnce(() -> {
          intakeMotorA.setSpeedRPM(0);
        }));
  }

  public Command intakeSlow() {
    return Commands.runOnce(() -> {
      isIntaking = true;
      intakeMotorA.setSpeedRPM(IntakeConstants.intakeSpeed);
    }, this);
  }

  public Command intakeFast() {
    return Commands.runOnce(() -> {
      isIntaking = true;
      intakeMotorA.setSpeedRPM(IntakeConstants.intakeSpeed * 1.35);
    }, this);
  }

  public Command stop() {
    return Commands.run(() -> {
      isIntaking = false;
      intakeMotorA.setSpeedRPM(0);
      ;
    }, this);
  }
}