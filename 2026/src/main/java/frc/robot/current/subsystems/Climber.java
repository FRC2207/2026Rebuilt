package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public static boolean isClimbingUp = false;
    public static boolean isClimbingDown = false;
    public static boolean isAtMax = false;
    public static boolean isAtMin = true;
    public static boolean climbError = false;

    public Side side;

    private final SendableChooser<Side> m_chooser = new SendableChooser<>();

    public enum Side {
        LEFT, RIGHT
    }

    public Climber() {
        SparkMaxConfig leftClimbMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();

        m_chooser.addOption("LEFT", Side.LEFT);
        m_chooser.addOption("RIGHT", Side.RIGHT);
        m_chooser.setDefaultOption("LEFT", Side.LEFT);

        SmartDashboard.putData("Climber Arm", m_chooser);

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

        if (getRotations(Side.RIGHT) <= -240 || getRotations(Side.LEFT) <= -240) {
            isAtMax = true;
            isAtMin = false;
        } else if (getRotations(Side.RIGHT) <= 10 || getRotations(Side.LEFT) <= 10) {
            isAtMin = true;
            isAtMax = false;
        } else {
            isAtMax = false;
            isAtMin = false;
        }
    }

    /**
     * Checks if the climb arms are fully retracted (inside the robot)
     * 
     * @return true if the climb arms are at the minimum position, false otherwise
     */
    public static boolean isWithinFrame() {
        return isAtMin;
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
        if (Pivot.isWithinFrame()) {
            climbError = false;
            return Commands.runOnce(() -> {
                isClimbingUp = true;
                isClimbingDown = false;
                leftClimbMotor.setMotorPercent(-climbSpeed * 1.25);
                rightClimbMotor.setMotorPercent(-climbSpeed * 1.25);
            }, this);
        } else {
            climbError = true;
            return Commands.none();
        }
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
        if (!Pivot.isWithinFrame()) {
            climbError = true;
            return Commands.none();
        } else {
            climbError = false;
            switch (side) {
                case LEFT:
                    return Commands.run(
                            () -> {
                                leftClimbMotor.setMotorPercent(-climbSpeed);
                            }).until(() -> getRotations(side) <= ClimbMax)
                            .finallyDo(() -> stop());
                case RIGHT:
                    return Commands.run(
                            () -> {
                                rightClimbMotor.setMotorPercent(-climbSpeed);
                            }).until(() -> getRotations(side) <= rightClimbMax)
                            .finallyDo(() -> stop());
                default:
                    return Commands.none();
            }
        }
    }

    /**
     * Individually lowers the climb arm to the lowest position (inside the robot)
     * 
     * @param side which selected climb arm
     * @return
     */
    public Command climbDownStowed() {
        switch (m_chooser.getSelected()) {
            case LEFT:
                return Commands.run(
                        () -> {
                            leftClimbMotor.setMotorPercent(climbSpeed);
                        }).until(() -> getRotations(m_chooser.getSelected()) >= ClimbMin)
                        .finallyDo(() -> stop());
            case RIGHT:
                return Commands.run(
                        () -> {
                            rightClimbMotor.setMotorPercent(climbSpeed);
                        }).until(() -> getRotations(m_chooser.getSelected()) >= ClimbMin)
                        .finallyDo(() -> stop());
            default:
                return Commands.none();
        }
    }

    public Command climbFlatMin(Side side) {
        switch (side) {
            case LEFT:
                return Commands.run(
                        () -> {
                            leftClimbMotor.setMotorPercent(climbSpeed);
                        }).until(() -> getRotations(side) >= prefferedMin)
                        .finallyDo(() -> stop());
            case RIGHT:
                return Commands.run(
                        () -> {
                            rightClimbMotor.setMotorPercent(climbSpeed);
                        }).until(() -> getRotations(side) >= prefferedMin)
                        .finallyDo(() -> stop());
            default:
                return Commands.none();
        }
    }

    public Command climbMaxBoth() {
        if (!Pivot.isWithinFrame()) {
            climbError = true;
            return Commands.none();
        } else {
            climbError = false;
            return Commands.run(
                    () -> {
                        isClimbingUp = true;
                        leftClimbMotor.setMotorPercent(-climbSpeed * 2);
                        rightClimbMotor.setMotorPercent(-climbSpeed * 2);
                    }).until(() -> getRotations(Side.LEFT) <= ClimbMax)
                    .finallyDo(() -> stop());
        }
    }

    public Command climbMinFlatBoth() {
        return Commands.run(
                () -> {
                    leftClimbMotor.setMotorPercent(climbSpeed);
                    rightClimbMotor.setMotorPercent(climbSpeed);
                }).until(() -> getRotations(Side.LEFT) >= prefferedMin || getRotations(Side.RIGHT) >= prefferedMin)
                .finallyDo(() -> stop());
    }
}
