package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.current.Constants.ClimberConstants.*;
import frc.robot.current.Constants;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorControllerIO;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Climber extends SubsystemBase {
    private MotorController leftClimbMotor;
    private MotorController rightClimbMotor;

    public boolean isClimbingUp = false;
    public boolean isClimbingDown = false;
    public boolean isAtMax = false;
    public boolean isAtMin = true;

    private enum Side {
        LEFT, RIGHT
    }

    public Climber() {
        SparkMaxConfig leftClimbMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();

        leftClimbMotorConfig.idleMode(IdleMode.kBrake);
        rightClimbMotorConfig.idleMode(IdleMode.kBrake);
        rightClimbMotorConfig.inverted(true);

        switch (Constants.currentMode) {
            case REAL:
                leftClimbMotor = new MotorController(new MotorIOSpark(leftClimbMotorID, leftClimbMotorConfig,
                        SparkType.SparkMax, MotorModel.NeoV1, EncoderType.BUILTIN_RELATIVE), "Climber/leftMotor");
                rightClimbMotor = new MotorController(new MotorIOSpark(rightClimbMotorID, rightClimbMotorConfig,
                        SparkType.SparkMax, MotorModel.NeoV1, EncoderType.BUILTIN_RELATIVE), "Climber/rightMotor");
                break;
            case SIM:
                leftClimbMotor = new MotorController(
                        new MotorIOSim(MotorModelSim.NeoV1, ControlType.Simple, 0, 0, 0, 0, 0, 1, 81),
                        "Climber/leftMotor");
                rightClimbMotor = new MotorController(
                        new MotorIOSim(MotorModelSim.NeoV1, ControlType.Simple, 0, 0, 0, 0, 0, 1, 81),
                        "Climber/rightMotor");
                break;
            default:
                // Blank IO for REPLAY
                leftClimbMotor = new MotorController(new MotorControllerIO() {
                }, "Climber/leftMotor");
                rightClimbMotor = new MotorController(new MotorControllerIO() {
                }, "Climber/rightMotor");
                break;
        }

        leftClimbMotor.resetEncoder();
        rightClimbMotor.resetEncoder();
    }

    public void periodic() {
        leftClimbMotor.updateInputs();
        rightClimbMotor.updateInputs();

        if (getRotations(Side.RIGHT) < -240 || getRotations(Side.LEFT) < -240) {
            isAtMax = true;
            isAtMin = false;
        } else if (getRotations(Side.RIGHT) < 10 || getRotations(Side.LEFT) < 10) {
            isAtMin = true;
            isAtMax = false;
        } else {
            isAtMax = false;
            isAtMin = false;
        }
    }

    public double getRotations(Side side) {
        switch (side) {
            case LEFT:
                return leftClimbMotor.getPositionRotations();
            case RIGHT:
                return rightClimbMotor.getPositionRotations();
            default:
                return 0.0;
        }
    }

    public Command climbUp() {
        return Commands.runOnce(() -> {
            isClimbingUp = true;
            isClimbingDown = false;
            leftClimbMotor.setMotorPercent(-climbSpeed * 1.25);
            rightClimbMotor.setMotorPercent(-climbSpeed * 1.25);
        }, this);
    }

    public Command climbDown() {
        return Commands.runOnce(() -> {
            isClimbingDown = true;
            isClimbingUp = false;
            leftClimbMotor.setMotorPercent(climbSpeed);
            rightClimbMotor.setMotorPercent(climbSpeed);
        }, this);
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            isClimbingUp = false;
            isClimbingDown = false;
            leftClimbMotor.setMotorPercent(0);
            rightClimbMotor.setMotorPercent(0);
        });
    }

    public Command climbMax(Side side) {
        switch (side) {
            case LEFT:
                return Commands.run(
                        () -> {
                            leftClimbMotor.setMotorPercent(-climbSpeed);
                        }).until(() -> getRotations(side) <= ClimbMax)
                        .finallyDo(() -> leftClimbMotor.setMotorPercent(0));
            case RIGHT:
                return Commands.run(
                        () -> {
                            rightClimbMotor.setMotorPercent(-climbSpeed);
                        }).until(() -> getRotations(side) <= rightClimbMax)
                        .finallyDo(() -> rightClimbMotor.setMotorPercent(0));
            default:
                return Commands.none();
        }
    }

    public Command climbMin(Side side) {
        switch (side) {
            case LEFT:
                return Commands.run(
                        () -> {
                            leftClimbMotor.setMotorPercent(climbSpeed);
                        }).until(() -> getRotations(side) >= prefferedMin)
                        .finallyDo(() -> leftClimbMotor.setMotorPercent(0));
            case RIGHT:
                return Commands.run(
                        () -> {
                            rightClimbMotor.setMotorPercent(climbSpeed);
                        }).until(() -> getRotations(side) >= prefferedMin)
                        .finallyDo(() -> rightClimbMotor.setMotorPercent(0));
            default:
                return Commands.none();
        }
    }

    public Command climbMaxBoth() {
        return Commands.run(
                () -> {
                    leftClimbMotor.setMotorPercent(-climbSpeed * 2);
                    rightClimbMotor.setMotorPercent(-climbSpeed * 2);
                }).until(() -> getRotations(Side.LEFT) <= ClimbMax)
                .finallyDo(() -> {
                    leftClimbMotor.setMotorPercent(0);
                    rightClimbMotor.setMotorPercent(0);
                });
    }

    public Command climbMinBoth() {
        return Commands.run(
                () -> {
                    leftClimbMotor.setMotorPercent(climbSpeed);
                    rightClimbMotor.setMotorPercent(climbSpeed);
                }).until(() -> getRotations(Side.LEFT) >= prefferedMin)
                .finallyDo(() -> {
                    leftClimbMotor.setMotorPercent(0);
                    rightClimbMotor.setMotorPercent(0);
                });
    }

    public Command levelOneClimb() {
        return Commands.sequence(climbMaxBoth(), Commands.waitSeconds(2), climbMinBoth());
    }

}
