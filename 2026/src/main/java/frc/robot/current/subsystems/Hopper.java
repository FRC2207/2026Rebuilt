package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Hopper {
    private SparkMaxConfig sparkConfig = new SparkMaxConfig();
    private MotorController motor;

    public Hopper() {
        switch (Constants.robot) {
            case "Real":
                motor = new MotorController(new MotorIOSpark(Constants.HopperConstants.motorID, sparkConfig, SparkType.SparkMax, MotorModel.NeoV1),
                        "Hopper");
                break;
            case "SIM":
                // Just don't use sim :)
                break;
            default:
                motor = new MotorController(new MotorIOSpark(Constants.HopperConstants.motorID, sparkConfig, SparkType.SparkMax, MotorModel.NeoV1),
                        "Hopper");
                break;
        }
    }

    public void periodic() {
        motor.updateInputs();
    }

    public void run() {
        motor.setMotorPercent(Constants.HopperConstants.motorSpeed);
    }

    public void stop() {
        motor.setMotorPercent(0);
    }
}
