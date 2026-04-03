package frc.robot.current.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public boolean outtaking = false;
    public boolean isInRange = false;

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

        // Set the prelearned distances (inches) with respective velocities (RPM)
        launchMap.put(55.0, 2900.0); // this is the only tested number
        launchMap.put(75.0, 3500.0);
        launchMap.put(100.0, 4000.0);
    }

    public void periodic() {
        highMotor.updateInputs();
        lowMotor.updateInputs();

        double distanceMeters = checkDistanceToHub();

        isInRange = (distanceMeters < 40 && distanceMeters > 300) ? false : true;

        // NOTE: using getSetpointRotations() because their is no setpoint retrival for
        // velocity control
        Logger.runEveryN(5,
                (Runnable) () -> Logger.recordOutput("Outtake/highMotor/setpointRPM", highMotor.getSetpoint()));
        Logger.runEveryN(5,
                (Runnable) () -> Logger.recordOutput("Outtake/lowMotor/setpointRPM", lowMotor.getSetpoint()));
        Logger.runEveryN(5,
                (Runnable) () -> Logger.recordOutput("Outtake/distanceFromHub", Units.metersToInches(distanceMeters)));
    }

    public Command timedLaunch(double seconds) {
        double highMotorSpeed = OuttakeConstants.velocityDefault * 1.25;
        double lowMotorSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                runOnce(() -> {
                    launchWithSpeeds(highMotorSpeed, lowMotorSpeed);
                }),
                Commands.waitSeconds(seconds),
                runOnce(() -> {
                    stop();
                }));
    }

    public Command continuousLaunch() {
        double highMotorSpeed = OuttakeConstants.velocityDefault * 1.25;
        double lowMotorSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                runOnce(() -> {
                    launchWithSpeeds(highMotorSpeed, lowMotorSpeed);
                }));
    }

    public void launchWithSpeeds(double highMotorSpeed, double lowMotorSpeed) {
        hopper.run();
        highMotor.setSpeedRPM(highMotorSpeed);
        lowMotor.setSpeedRPM(lowMotorSpeed);
        outtaking = true;
    }

    /**
     * The launcher sometimes gets balls stuck in the front. Ideally, this code will
     * spit that ball out if it occurs. Otherwise it will run the variable launch
     * map
     */
    public Command launcherPro() {
        if (lowMotor.getCurrent() < motorStalledCurrent) {
            return variableLaunchMap();
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
                    double velocity = getVelocityTarget(Units.metersToInches(checkDistanceToHub()));
                    launchWithSpeeds(velocity * 1.25, velocity);
                }));
    }

    public Command fancyLaunch() {
        Command startup = Commands.sequence(
            Commands.runOnce(() -> {
                lowMotor.setSpeedRPM(getVelocityTarget(Units.metersToInches(checkDistanceToHub())));
                hopper.runBackwards();
                }, this), 
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                highMotor.setSpeedRPM(-1500);
            }, this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                highMotor.setSpeedRPM(getVelocityTarget(Units.metersToInches(checkDistanceToHub())));
            }, this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                hopper.run();
            }, this)
            );
        
        if (lowMotor.getCurrent() < motorStalledCurrent) {
            return startup;
        } else {
            return Commands.sequence(
                    runOnce(() -> {
                        lowMotor.setSpeedRPM(OuttakeConstants.velocityDefault * -1);
                    }),
                    Commands.waitSeconds(0.1),
                    startup);
        }
    }

    public Command manualTuningLaunch() {
        return Commands.run(() -> {
            double velocity = manualShooterSetpoint.get();
            launchWithSpeeds(velocity, 500);
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
                    Logger.runEveryN(5, (Runnable) () -> Logger.recordOutput("Outtake/ballVelocity", ball_velocity));
                    double velocity = (ball_velocity * (60 / (0.0254 * Math.PI * 3))) + 200;
                    launchWithSpeeds(velocity + 150, velocity);
                }, this));
    }

    /** Stops all the motors */
    public Command stop() {
        return Commands.runOnce(() -> {
            hopper.stop();
            highMotor.setSpeedRPM(0);
            lowMotor.setSpeedRPM(0);
            outtaking = false;
        }, this);
    }

    public double getVelocityTarget(double distance) {
        return launchMap.get(distance);
    }

    /** Checks the distance from the bot to the target */
    public double checkDistance(Pose2d target) {
        double value = swerve.getPose().getTranslation().getDistance(target.getTranslation());
        return value;
    }

    public double checkDistanceToHub() {
        return checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                ? FieldConstants.Elements.redHubPose
                : FieldConstants.Elements.blueHubPose);
    }
}
