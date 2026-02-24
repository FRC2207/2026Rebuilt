package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.current.subsystems.swerveDrive.Drive;

public class ObjectVisionIODetection implements ObjectVisionIO {
    private Drive swerve;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("YellowDetectorFieldData");
    private DoubleArraySubscriber xPointsSubscriberField;
    private DoubleArraySubscriber yPointsSubscriberField;

    private DoubleArraySubscriber xPointsSubscriberHopper;
    private DoubleArraySubscriber yPointsSubscriberHopper;
    private IntegerSubscriber fuelCountSubscriber;
    private String subsystemName;
    private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2.0, 1.0); // m/s, m/s^2

    public ObjectVisionIODetection(Drive drive, String subsystemName) {
        this.swerve = drive;
        this.subsystemName = subsystemName;

        xPointsSubscriberField = table.getDoubleArrayTopic("fuelPointsXField").subscribe(new double[] {});
        yPointsSubscriberField = table.getDoubleArrayTopic("fuelPointsYField").subscribe(new double[] {});
        // Field versus hopper
        xPointsSubscriberHopper = table.getDoubleArrayTopic("fuelPointsXHopper").subscribe(new double[] {});
        yPointsSubscriberHopper = table.getDoubleArrayTopic("fuelPointsYHopper").subscribe(new double[] {});
        // Fuel count stuff
        fuelCountSubscriber = table.getIntegerTopic("detectedObjects").subscribe(-1);
    }

    private double[] getFuelPointsX() {
        return xPointsSubscriberField.get();
    }

    private double[] getFuelPointsY() {
        return yPointsSubscriberField.get();
    }

    @Override
    public Pose2d[] subsystemData(String subsystem) {
        if (subsystemName != "field") {
            switch (subsystem) {
                case "hopper":
                    return doubleArraysToPose2dArray(xPointsSubscriberHopper.get(), yPointsSubscriberHopper.get());
                case "field":
                default:
                    return doubleArraysToPose2dArray(xPointsSubscriberField.get(), yPointsSubscriberField.get());
            }
        } else {
            return null;
        }
    }

    private int getDetectedBallCount() {
        return (int) fuelCountSubscriber.get();
    }

    private Pose2d[] doubleArraysToPose2dArray(double[] xArray, double[] yArray) {
        Pose2d[] poses = new Pose2d[xArray.length];
        for (int i = 0; i < xArray.length; i++) {
            poses[i] = new Pose2d(xArray[i], yArray[i], new Rotation2d(0.0));
        }
        return poses;
    }

    @Override
    public Boolean seesObject() {
        if (subsystemName == "hopper") {
            if (xPointsSubscriberHopper.get().length != 0 && yPointsSubscriberField.get().length != 0) {
                return true;
            } else {
                return false;
            }
        } else {
            if (xPointsSubscriberField.get().length != 0 && yPointsSubscriberField.get().length != 0) {
                return true;
            } else {
                return false;
            }
        }
    }

    /**
     * My first java doc
     * this should return a robot relative Trajectory idk how to implement that
     */
    @Override
    public Trajectory getPath() {
        if (subsystemName != "field") {
            double[] fuelPointsX = getFuelPointsX();
            double[] fuelPointsY = getFuelPointsY();

            if (fuelPointsX.length != fuelPointsY.length) {
                // This shouldn't happen but just in case
                return null;
            }

            trajectoryConfig.setKinematics(swerve.getSwerveKinematics());

            Pose2d startPosition = swerve.getPose();
            Pose2d endPosition;
            // Im gonna set this up two ways, and uncomment the best one
            // 1. Endoint is like 4 meters in front of the robot
            endPosition = swerve.getPose().transformBy(new Transform2d(4, 0, swerve.getRotation()));
            // 2. Final point which will be the point farthest from the robot (my python
            // code will handle that)
            // endPosition = new Pose2d(fuelPointsX[fuelPointsX.length - 1],
            // fuelPointsY[fuelPointsY.length - 1], new Rotation2d());

            List<Translation2d> translationPoints = new ArrayList<>();
            for (int i = 0; i < fuelPointsX.length; i += 1) {
                translationPoints.add(new Translation2d(fuelPointsX[i], fuelPointsY[i + 1]));
            }

            // TODO: implement object avoidance here but idk how
            return TrajectoryGenerator.generateTrajectory(
                    startPosition,
                    translationPoints,
                    endPosition,
                    trajectoryConfig).relativeTo(swerve.getPose());
        } else {
            return null;
        }
    }

    @Override
    public void updateInputs(objectVisionIOInputs inputs) {
        inputs.ballsDetected = getDetectedBallCount();
    }
}