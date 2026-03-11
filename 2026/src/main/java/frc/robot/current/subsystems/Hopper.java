package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSparkMax;

public class Hopper {
    private SparkMaxConfig sparkConfig = new SparkMaxConfig();
    private MotorController motor;

    public Hopper() {
        switch (Constants.robot) {
            case "Real":
                motor = new MotorController(new MotorIOSparkMax(Constants.HopperConstants.motorID, sparkConfig, 35),
                        "Hopper");
                break;
            case "SIM":
                // Just don't use sim :)
                break;
            default:
                motor = new MotorController(new MotorIOSparkMax(Constants.HopperConstants.motorID, sparkConfig, 35),
                        "Hopper");
                break;
        }
    }

    public void periodic() {
        motor.updateInputs();
    }

    public void run() {
        motor.setPercent(Constants.HopperConstants.motorSpeed);
    }

    public void stop() {
        motor.setPercent(0);
    }
}
