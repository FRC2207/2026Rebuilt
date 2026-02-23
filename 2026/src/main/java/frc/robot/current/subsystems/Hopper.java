package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;

public class Hopper {
    private SparkMaxConfig sparkConfig = new SparkMaxConfig();
    private MotorController motor;

    public Hopper(){
        switch (Constants.robot) {
            case "Real":
                motor = new MotorController(new MotorIOSparkMax(Constants.HopperConstants.motorID, sparkConfig,35), "Hopper", "1");
            case "SIM":
                // Just don't use sim :)
            default:
                motor = new MotorController(new MotorIOSparkMax(Constants.HopperConstants.motorID, sparkConfig,35), "Hopper", "1");
        }
    }

    public void periodic() {
        motor.updateInputs();
    }

    public Command run() {
        return Commands.run(
            () -> {
                motor.setPercent(Constants.HopperConstants.motorSpeed);
            });
    }

    public Command stop() {
        return Commands.run(
            () -> {
                motor.setPercent(0);
            });
    }
}
