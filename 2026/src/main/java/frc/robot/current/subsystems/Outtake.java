package frc.robot.current.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import frc.robot.lib.motors.motorController.MotorIOSpark.EncoderType;
import frc.robot.lib.motors.motorController.MotorIOSim;
import frc.robot.lib.motors.motorController.MotorIOSpark.MotorModel;
import frc.robot.lib.motors.motorController.MotorIOSpark.SparkType;
import frc.robot.lib.motors.motorController.MotorIOSpark;
import frc.robot.lib.motors.motorController.MotorIOSim.ControlType;
import frc.robot.lib.motors.motorController.MotorIOSim.MotorModelSim;
import frc.robot.lib.ObjectVision.ObjectVision;

public class Outtake extends SubsystemBase {
    private MotorController highMotor;
    private MotorController lowMotor;

    private Drive swerve;
    private Hopper hopper;
    private Boolean hopperEmpty = false;

    private InterpolatingDoubleTreeMap launchMap = new InterpolatingDoubleTreeMap();

    private final int highMotorId = OuttakeConstants.highMotorId;
    private final int lowMotorId = OuttakeConstants.lowMotorId;
    private ObjectVision objectVision;
    private EncoderType encoderType = EncoderType.BUILTIN_RELATIVE;

    public Outtake(Drive drive, Hopper hopper, ObjectVision objectVision) {
        this.swerve = drive;
        this.hopper = hopper;
        this.objectVision = objectVision;
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
                        new MotorControllerIO() {},
                        "Outtake/highMotor");
                lowMotor = new MotorController(
                        new MotorControllerIO() {},
                        "Outtake/lowMotor");
                break;
        }

        // Set the prelearned distances (inches) with respective velocities (RPM)
        launchMap.put(20.0, 1800.0);
        launchMap.put(40.0, 2200.0);
        launchMap.put(60.0, 2600.0);
        launchMap.put(76.0, 3000.0);
        launchMap.put(80.0, 3100.0);
        launchMap.put(100.0, 3200.0);
        launchMap.put(120.0, 3300.0);
        launchMap.put(140.0, 3400.0);
        launchMap.put(160.0, 3500.0);
        launchMap.put(180.0, 3600.0);
        launchMap.put(200.0, 3700.0);
    }

    public void periodic() {
        highMotor.updateInputs();
        lowMotor.updateInputs();

        // NOTE: using getSetpointRotations() because their is no setpoint retrival for velocity control
        Logger.recordOutput("Outtake/highMotor/setpointRPM", highMotor.getSetpoint());
        Logger.recordOutput("Outtake/lowMotor/setpointRPM", lowMotor.getSetpoint());
    }

    public Command timedLaunch(double seconds) {
        double motorOneSpeed = OuttakeConstants.velocityDefault * 1.25;
        double motorTwoSpeed = OuttakeConstants.velocityDefault;

        return Commands.sequence(
                runOnce(() -> {
                    hopper.run();
                    highMotor.setSpeedRPM(motorOneSpeed);
                    lowMotor.setSpeedRPM(motorTwoSpeed);
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
                }));
    }

    // Runs the launcher at variable RPM in relation to distance from the hub.
    // Motors stop when the hopper is empty
    public Command variableLaunchMap() {
        return Commands.sequence(
                run(() -> {
                    double velocity = getVelocityTarget(
                            checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                                    ? FieldConstants.Elements.redHubPose
                                    : FieldConstants.Elements.blueHubPose));
                    hopper.run();
                    highMotor.setSpeedRPM(velocity * 1.25);
                    lowMotor.setSpeedRPM(velocity);
                }));
    }

    public Command variableLaunchEquation() {
        return Commands.run(() -> {
            double distanceRaw = checkDistance((DriverStation.getAlliance().get() == Alliance.Red)
                    ? FieldConstants.Elements.redHubPose
                    : FieldConstants.Elements.blueHubPose);
            double distance = distanceRaw - 0.5;
            // The velocity the ball needs to be at to hit the target in m/s
            double ball_velocity = (Math.sqrt((23.0526875 * Math.pow(distance, 2))/(distance + (-1.482/4.7046))))/0.978147;
            double velocity = (ball_velocity * (60/ (0.0254 * Math.PI * 3))) + 200;
            Logger.recordOutput("Outtake/ballVelocity", ball_velocity);
            Logger.recordOutput("Outtake/distance", distance);
            // System.out.println("Velocity: " + velocity);
            // System.out.println("Distance: " + distance);
            hopper.run();
            highMotor.setSpeedRPM(velocity + 150);
            lowMotor.setSpeedRPM(velocity);
        }, this);
    }

    /** Stops all the motors */
    public Command stop() {
        return Commands.runOnce(() -> {
            hopper.stop();
            highMotor.setSpeedRPM(0);
            lowMotor.setSpeedRPM(0);
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
