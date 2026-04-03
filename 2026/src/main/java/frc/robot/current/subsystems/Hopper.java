package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorControllerIO;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Hopper extends SubsystemBase {
    private SparkMaxConfig sparkConfig = new SparkMaxConfig();
    private MotorController motor;

    public Hopper() {
        sparkConfig.smartCurrentLimit(20);
        
        switch (Constants.currentMode) {
            case REAL:
                motor = new MotorController(new MotorIOSpark(Constants.HopperConstants.motorID, sparkConfig, SparkType.SparkMax, MotorModel.NeoV1, EncoderType.BUILTIN_RELATIVE),
                        "Hopper");
                break;
            case SIM:
                motor = new MotorController(
                        new MotorIOSim(MotorModelSim.Neo550, ControlType.Simple, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0),
                        "Hopper");
                break;
            default:
            // Blank IO for REPLAY
                motor = new MotorController(new MotorControllerIO() {},
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

    public void runBackwards(){
        motor.setMotorPercent(-Constants.HopperConstants.motorSpeed);
    }

    public void stop() {
        motor.setMotorPercent(0);
    }
}
