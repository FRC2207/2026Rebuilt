package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.IntakeConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.velocityController.VelocityController;
import frc.robot.lib.motors.velocityController.VelocityIOSparkFlex;

public class Intake extends SubsystemBase {
  private VelocityController intakeMotor;

  private final int intakeMotorId = Constants.IntakeConstants.intakeID;
  private final String robotType = Constants.robot;

  public Boolean isIntaking = false;
  
  public Intake(Drive drive) {

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.inverted(true);
    intakeConfig.smartCurrentLimit(30);
    intakeConfig.closedLoop
                .p(IntakeConstants.kP)
                .i(IntakeConstants.kI)
                .d(IntakeConstants.kD)
            .feedForward // Set Feedforward gains for the velocity controller
                .kS(IntakeConstants.kS) // Static gain (volts)
                .kV(IntakeConstants.kV) // Velocity gain (volts per RPM)
                .kA(IntakeConstants.kA); // Acceleration gain (volts per RPM/s)

    switch (robotType) {
      case "Real":
        intakeMotor = new VelocityController(new VelocityIOSparkFlex(intakeMotorId, intakeConfig), "Outtake", "1");
        break;
      case "SIM":
        // Just don't use sim.

        break;
      default:
        intakeMotor = new VelocityController(new VelocityIOSparkFlex(intakeMotorId, intakeConfig), "Outtake", "1");
        break;
    }
  }

  public void periodic() {
    intakeMotor.updateInputs();
  }

  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public Command spit() { 
    double percent = 2000;

    return Commands.sequence(
        runOnce(() -> {
          intakeMotor.setSpeed(percent);
        }),
        Commands.waitSeconds(.5),
        runOnce(() -> {
          intakeMotor.setVoltage(0);
        }));
  }

  public Command intake() {
    return Commands.runOnce(() -> {
      isIntaking = true;
      intakeMotor.setSpeed(IntakeConstants.intakeSpeed);
    }, this);
  }

  public Command stop() {
    return Commands.run(() -> {
      isIntaking = false;
      intakeMotor.setVoltage(0);
    }, this);
  }
}