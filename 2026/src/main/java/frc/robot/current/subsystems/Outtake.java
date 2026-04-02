package frc.robot.current.subsystems;

import org.ejml.equation.VariableType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.current.Constants.OuttakeConstants;
import frc.robot.current.FieldConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.motors.motorController.MotorController;
import frc.robot.lib.motors.motorController.MotorControllerIO;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;

public class Outtake extends SubsystemBase {
    private MotorController highMotor;
    private MotorController lowMotor;

    private Drive swerve;
    private Hopper hopper;
    private boolean hopperEmpty = false;
    public boolean outtaking = false;
    public boolean isInRage = false;
    private boolean staticLaunch = false;

    private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

    private final int highMotorId = OuttakeConstants.highMotorId;
    private final int lowMotorId = OuttakeConstants.lowMotorId;
    private EncoderType encoderType = EncoderType.BUILTIN_RELATIVE;
    private final double motorStalledCurrent = 30.0; // Current threshold to determine if the motor is stalled

    private final LoggedNetworkNumber manualShooterSetpoint = new LoggedNetworkNumber("/Outtake/ShooterSetpoint",
            1000.0);

    public Outtake(Drive drive, Hopper hopper) {
        this.swerve = drive;
        this.hopper = hopper;

        SparkFlexConfig lowConfig = new SparkFlexConfig();
        SparkFlexConfig highConfig = new SparkFlexConfig();
        lowConfig.inverted(false);
        highConfig.inverted(true);
        lowConfig.smartCurrentLimit(70);
        highConfig.smartCurrentLimit(70);
        lowConfig.closedLoop
                .p(OuttakeConstants.kP)
                .i(OuttakeConstants.kI)
                .d(OuttakeConstants.kD).feedForward // Set Feedforward gains for the velocity controller
                .kS(OuttakeConstants.kS) // Static gain (volts)
                .kV(OuttakeConstants.kV) // Velocity gain (volts per RPM)
                .kA(OuttakeConstants.kA); // Acceleration gain (volts per RPM/s)
        highConfig.closedLoop
                .p(OuttakeConstants.kP)
                .i(OuttakeConstants.kI)
                .d(OuttakeConstants.kD).feedForward // Set Feedforward gains for the velocity controller
                .kS(OuttakeConstants.kS) // Static gain (volts)
                .kV(OuttakeConstants.kV) // Velocity gain (volts per RPM)
                .kA(OuttakeConstants.kA); // Acceleration gain (volts per RPM/s)

        switch (Constants.currentMode) {
            case REAL:
                highMotor = new MotorController(
                        new MotorIOSpark(highMotorId, highConfig, SparkType.SparkFlex, MotorModel.Vortex, encoderType),
                        "Outtake/highMotor");
                lowMotor = new MotorController(
                        new MotorIOSpark(lowMotorId, lowConfig, SparkType.SparkFlex, MotorModel.Vortex, encoderType),
                        "Outtake/lowMotor");

                break;
            case SIM:
                highMotor = new MotorController(new MotorIOSim(MotorModelSim.Vortex, ControlType.Velocity,
                        OuttakeConstants.kSim_P, OuttakeConstants.kSim_I, OuttakeConstants.kSim_D,
                        OuttakeConstants.kSim_S, OuttakeConstants.kSim_V, OuttakeConstants.kSim_TopMOI,
                        OuttakeConstants.kSim_TopGearReduction), "Outtake/highMotor");

                lowMotor = new MotorController(new MotorIOSim(MotorModelSim.Vortex, ControlType.Velocity,
                        OuttakeConstants.kSim_P, OuttakeConstants.kSim_I, OuttakeConstants.kSim_D,
                        OuttakeConstants.kSim_S, OuttakeConstants.kSim_V, OuttakeConstants.kSim_BottomMOI,
                        OuttakeConstants.kSim_BottomGearReduction), "Outtake/lowMotor");

                break;
            default:
                // Blank IO for REPLAY
                highMotor = new MotorController(
                        new MotorControllerIO() {
                        },
                        "Outtake/highMotor");
                lowMotor = new MotorController(
                        new MotorControllerIO() {
                        },
                        "Outtake/lowMotor");
                break;

        }

        SmartDashboard.putBoolean("Static Launch Speed", staticLaunch);

        // Set the prelearned distances (inches) with respective velocities (RPM)
        launchMap.put(55.0, 2800.0); // this is the only tested number
        launchMap.put(75.0, 3500.0);
        launchMap.put(100.0, 4000.0);
    }

    public void periodic() {
        highMotor.updateInputs();
        lowMotor.updateInputs();

        if (checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                ? FieldConstants.Elements.redHubPose
                : FieldConstants.Elements.blueHubPose) < 40
                && checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                        ? FieldConstants.Elements.redHubPose
                        : FieldConstants.Elements.blueHubPose) > 300) {
            isInRage = false;
        } else {
            isInRage = true;
        }

        // NOTE: using getSetpointRotations() because their is no setpoint retrival for
        // velocity control
        Logger.runEveryN(5,
                (Runnable) () -> Logger.recordOutput("Outtake/highMotor/setpointRPM", highMotor.getSetpoint()));
        Logger.runEveryN(5,
                (Runnable) () -> Logger.recordOutput("Outtake/lowMotor/setpointRPM", lowMotor.getSetpoint()));
        Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Outtake/distanceFromHub", Units.metersToInches(
                checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                        ? FieldConstants.Elements.redHubPose
                        : FieldConstants.Elements.blueHubPose))));
    }

    public Command timedLaunch(double seconds) {
        double motorOneSpeed = OuttakeConstants.velocityDefault * 1.25;
        double motorTwoSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                runOnce(() -> {
                    hopper.run();
                    highMotor.setSpeedRPM(motorOneSpeed);
                    lowMotor.setSpeedRPM(motorTwoSpeed);
                    outtaking = true;
                }),
                Commands.waitSeconds(seconds),
                runOnce(() -> {
                    stop();
                }));
    }

    public Command continuousLaunch() {
        double motorOneSpeed = OuttakeConstants.velocityDefault * 1.25;
        double motorTwoSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                runOnce(() -> {
                    hopper.run();
                    highMotor.setSpeedRPM(motorOneSpeed);
                    lowMotor.setSpeedRPM(motorTwoSpeed);
                    outtaking = true;
                }));
    }

    /**
     * The launcher sometimes gets balls stuck in the front. Ideally, this code will
     * spit that ball out if it occurs. Otherwise it will run the variable launch
     * map
     */
    public Command launcherPro() {
        if (lowMotor.getCurrent() < motorStalledCurrent) {
            if (staticLaunch) {
            return continuousLaunch();
            } else {
                return variableLaunchEquation();
            }
        } else {
            return Commands.sequence(
                    runOnce(() -> {
                        lowMotor.setSpeedRPM(OuttakeConstants.velocityDefault * -1);
                    }),
                    Commands.waitSeconds(0.1),
                    variableLaunchMap());
        }
    }

    // Runs the launcher at variable RPM in relation to distance from the hub.
    // Motors stop when the hopper is empty
    public Command variableLaunchMap() {
        return Commands.sequence(
                run(() -> {
                    double distance = Units.metersToInches(
                            checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                                    ? FieldConstants.Elements.redHubPose
                                    : FieldConstants.Elements.blueHubPose));
                    double velocity = getVelocityTarget(distance);
                    Logger.recordOutput("Outtake/distance", distance);
                    Logger.recordOutput("Outtake/velocitySet", velocity);
                    hopper.run();
                    highMotor.setSpeedRPM(velocity * 1.25);
                    lowMotor.setSpeedRPM(velocity);
                    outtaking = true;
                }));
    }

    public Command manualTuningLaunch() {
        return Commands.run(() -> {
            double velocity = manualShooterSetpoint.get();
            hopper.run();
            highMotor.setSpeedRPM(velocity);
            lowMotor.setSpeedRPM(500);
        }, this);
    }

    public Command variableLaunchEquation() {
        return Commands.parallel(Commands.runOnce(() -> outtaking = true),
                Commands.run(() -> {
                    double distanceRaw = checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                            ? FieldConstants.Elements.redHubPose
                            : FieldConstants.Elements.blueHubPose);
                    double distance = distanceRaw - 0.5;
                    // The velocity the ball needs to be at to hit the target in m/s
                    double ball_velocity = (Math
                            .sqrt((23.0526875 * Math.pow(distance, 2)) / (distance + (-1.482 / 4.7046))))
                            / 0.978147;
                    double velocity = (ball_velocity * (60 / (0.0254 * Math.PI * 3))) + 200;
                    Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Outtake/ballVelocity", ball_velocity));
                    Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Outtake/distance", distance));
                    hopper.run();
                    highMotor.setSpeedRPM(velocity);
                    lowMotor.setSpeedRPM(velocity * 1.15);
                }, this));
    }

    /** Stops all the motors */
    public Command stop() {
        return Commands.runOnce(() -> {
            hopper.stop();
            highMotor.setSpeedRPM(0);
            lowMotor.setSpeedRPM(0);
            outtaking = false;
        },
                this);
    }

    public double getVelocityTarget(double distance) {
        return launchMap.get(distance);
    }

    public Boolean getHopperEmpty() {
        return hopperEmpty;
    }

    /** Checks the distance from the bot to the target */
    public double checkDistance(Pose2d target) {
        double value = swerve.getPose().getTranslation().getDistance(target.getTranslation());
        return value;
    }
}
