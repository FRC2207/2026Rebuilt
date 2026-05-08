package frc.robot.current.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.current.Constants.ClimberConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

    private final LoggedDashboardChooser<Side> m_chooser = new LoggedDashboardChooser<Side>("Climber Side");

    public enum Side {
        LEFT, RIGHT
    }

    public Climber() {
        SparkMaxConfig leftClimbMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();

        m_chooser.addOption("RIGHT", Side.RIGHT);
        m_chooser.addDefaultOption("LEFT", Side.LEFT);

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

        SmartDashboard.putData("Reset Left",
                Commands.runOnce(() -> resetEncoder(Side.LEFT)));
        SmartDashboard.putData("Reset Right",
                Commands.runOnce(() -> resetEncoder(Side.RIGHT)));

        resetEncoder(Side.LEFT);
        resetEncoder(Side.RIGHT);
    }

    public void periodic() {
        leftClimbMotor.updateInputs();
        rightClimbMotor.updateInputs();

        if (getRightRotations() <= -240 || getLeftRotations() <= -240) {
            isAtMax = true;
            isAtMin = false;
        } else if (getRightRotations() <= 10 || getLeftRotations() <= 10) {
            isAtMin = true;
            isAtMax = false;
        } else {
            isAtMax = false;
            isAtMin = false;
        }
    }

    // Utility methods for the climber subsystem.

    /**
     * Resets the encoders of either climb motor
     */
    public void resetEncoder(Side side) {
        switch (side) {
            case LEFT:
                leftClimbMotor.resetEncoder();
                break;
            case RIGHT:
                rightClimbMotor.resetEncoder();
                break;
        }
    }

    /**
     * Raises the left arm up at the default speed, multiplied by 2
     */
    private void setLeftUp() {
        if (Pivot.isWithinFrame()) {
            isClimbingUp = true;
            leftClimbMotor.setMotorPercent(-climbSpeed * 2);
        } else {
            climbError = true;
        }
    }

    /**
     * Raises the right arm up at the default speed, multiplied by 2
     */
    private void setRightUp() {
        if (Pivot.isWithinFrame()) {
            isClimbingUp = true;
            rightClimbMotor.setMotorPercent(-climbSpeed * 2);
        } else {
            climbError = true;
        }
    }

    /**
     * Lowers the left arm down at the default speed
     */
    private void setLeftDown() {
        isClimbingDown = true;
        leftClimbMotor.setMotorPercent(climbSpeed);
    }

    /**
     * Lowers the right arm down at the default speed
     */
    private void setRightDown() {
        isClimbingDown = true;
        rightClimbMotor.setMotorPercent(climbSpeed);
    }

    /**
     * Stops the left climb arm
     */
    private void setLeftStop() {
        isClimbingDown = false;
        isClimbingUp = false;
        leftClimbMotor.setMotorPercent(0);
    }

    /**
     * Stops the right climb arm
     */
    private void setRightStop() {
        isClimbingDown = false;
        isClimbingUp = false;
        rightClimbMotor.setMotorPercent(0);
    }

    /**
     * 
     * @return The current position of the left climb arm in rotations.
     */
    public double getLeftRotations() {
        return leftClimbMotor.getPositionRotations();
    }

    /**
     * 
     * @return The current position of the right climb arm in rotations.
     */
    public double getRightRotations() {
        return rightClimbMotor.getPositionRotations();
    }

    /**
     * Checks if the climb arms are fully retracted (inside the robot)
     * 
     * @return true if the climb arms are at the minimum position, false otherwise
     */
    public static boolean isWithinFrame() {
        return isAtMin;
    }

    // End of the utility methods.

    /**
     * @return A command to raise both climb arms
     */
    public Command climbUpBoth() {
        return Commands.run(() -> {
                setLeftUp();
                setRightUp();        
        }, this).until(() -> climbError);
    }

    /**
     * @return A command to lower both climb arms
     */
    public Command climbDownBoth() {
        return Commands.run(() -> {
            setLeftDown();
            setRightDown();
        }, this);
    }

    public Command climbDownIndividual() {
        Side selected;
        if (m_chooser.get() == null) {
            selected = Side.LEFT;
        } else {
            selected = m_chooser.get();
        }
        switch (selected) {
            case LEFT:
                return Commands.run(() -> setLeftDown());
            case RIGHT:
                return Commands.run(() -> setRightDown());
            default:
                return Commands.none();
        }
    }

    /**
     * Arm is decided by a sendable chooser.
     * 
     * @return a command to individually raise the climb arms to the maximum
     *         position (fully extended).
     */
    public Command climbMaxIndividual() {
        switch (m_chooser.get()) {
            case LEFT:
                try {
                    return Commands.run(
                            () -> {
                                setLeftUp();
                            }).until(() -> getLeftRotations() <= legalMax)
                            .finallyDo(() -> {
                                setLeftStop();
                                setRightStop();
                            });
                } catch (IllegalStateException e) {
                    climbError = true;
                    return Commands.none();
                }
            case RIGHT:
                try {
                    return Commands.run(
                            () -> {
                                setRightUp();
                            }).until(() -> getRightRotations() <= legalMax)
                            .finallyDo(() -> {
                                setLeftStop();
                                setRightStop();
                            });
                } catch (IllegalStateException e) {
                    climbError = true;
                    return Commands.none();
                }
            default:
                return Commands.none();
        }
    }

    /**
     * 
     * @return A command to raise both climb arms to the maximum position (fully
     *         extended).
     */
    public Command climbMaxBoth() {
        return Commands.run(() -> {
                setLeftUp();
                setRightUp();
        }, this)
        .until(() -> (getLeftRotations() <= legalMax || getRightRotations() <= legalMax))
        .finallyDo(() -> {
            setLeftStop();
            setRightStop();
        });
    }

    /**
     * Arm is decided by a sendable chooser.
     * 
     * @return a Command to individually lower the climb arms to the absolute lowest
     *         position (inside the robot).
     */
    public Command climbStowedIndividual() {
        switch (m_chooser.get()) {
            case LEFT:
                return Commands.run(
                        () -> {
                            setLeftDown();
                        }).until(() -> getLeftRotations() >= absoluteMin)
                        .finallyDo(() -> {
                            setLeftStop();
                            setRightStop();
                        });
            case RIGHT:
                return Commands.run(
                        () -> {
                            setRightDown();
                        }).until(() -> getRightRotations() >= absoluteMin)
                        .finallyDo(() -> {
                            setLeftStop();
                            setRightStop();
                        });
            default:
                return Commands.none();
        }
    }

    /**
     * 
     * @return A command to lower both climb arms to the absolute lowest position
     *         (inside the robot)
     */
    public Command climbStowedBoth() {
        return Commands.run(
                () -> {
                    setLeftDown();
                    setRightDown();
                }).until(() -> getLeftRotations() >= absoluteMin || getRightRotations() >= absoluteMin)
                .finallyDo(() -> {
                    setLeftStop();
                    setRightStop();
                });
    }

    /**
     * Arm is decided by a sendable chooser.
     * 
     * @return a Command to individually lower the climb arms to the lowest flat
     *         position.
     */
    public Command climbFlatIndividual() {
        switch (m_chooser.get()) {
            case LEFT:
                return Commands.run(
                        () -> {
                            setLeftDown();
                        }).until(() -> getLeftRotations() >= flatMin)
                        .finallyDo(() -> {
                            setLeftStop();
                            setRightStop();
                        });
            case RIGHT:
                return Commands.run(
                        () -> {
                            setRightDown();
                        }).until(() -> getRightRotations() >= flatMin)
                        .finallyDo(() -> {
                            setLeftStop();
                            setRightStop();
                        });
            default:
                return Commands.none();
        }
    }

    /**
     * Arm is decided by a sendable chooser.
     * 
     * @return a Command to lower both climb arms to the lowest flat position.
     */
    public Command climbFlatBoth() {
        return Commands.run(
                () -> {
                    setLeftDown();
                    setRightDown();
                }).until(() -> getLeftRotations() >= flatMin || getRightRotations() >= flatMin)
                .finallyDo(() -> {
                    setLeftStop();
                    setRightStop();
                }); 
        }
}