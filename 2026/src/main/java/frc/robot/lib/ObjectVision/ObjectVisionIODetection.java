package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.current.subsystems.swerveDrive.Drive;

public class ObjectVisionIODetection implements ObjectVisionIO {
    private Drive swerve;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("YellowDetectorFieldData");
    private DoubleArraySubscriber xPointsSubscriberField;
    private DoubleArraySubscriber yPointsSubscriberField;

    private BooleanSubscriber hopperSubscriber;
    private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2.0, 1.0); // m/s, m/s^2

    public ObjectVisionIODetection(Drive drive) {
        this.swerve = drive;

        xPointsSubscriberField = table.getDoubleArrayTopic("field_positions_X").subscribe(new double[] {});
        yPointsSubscriberField = table.getDoubleArrayTopic("field_positions_Y").subscribe(new double[] {});
        // Field versus hopper

        hopperSubscriber = table.getBooleanTopic("hopper_sees_object").subscribe(false);
    }

    private double[] getFuelPointsX() {
        return xPointsSubscriberField.get();
    }

    private double[] getFuelPointsY() {
        return yPointsSubscriberField.get();
    }

    // @Override
    // public Pose2d[] subsystemData(String subsystem) {
    //     if (subsystemName != "field") {
    //         switch (subsystem) {
    //             case "hopper":
    //                 return doubleArraysToPose2dArray(xPointsSubscriberHopper.get(), yPointsSubscriberHopper.get());
    //             case "field":
    //             default:
    //                 return doubleArraysToPose2dArray(xPointsSubscriberField.get(), yPointsSubscriberField.get());
    //         }
    //     } else {
    //         return null;
    //     }
    // }

    // private int getDetectedBallCount() {
    //     return (int) fuelCountSubscriber.get();
    // }

    private List<Translation2d> doubleArraysToTranslation2dList(double[] xArray, double[] yArray) {
        List<Translation2d> poses = new ArrayList<>();
        int length = Math.min(xArray.length, yArray.length);
        for (int i = 0; i < length; i++) {
            poses.add(new Translation2d(xArray[i], yArray[i]));
        }
        return poses;
    }

    @Override
    public Boolean hopperSeesObject() {
        if (hopperSubscriber.exists()) {
            return hopperSubscriber.get();
        } else {
            return null;
        }
    }

    private List<Translation2d> getFieldPositions() {
        double[] fuelPointsX = getFuelPointsX();
        double[] fuelPointsY = getFuelPointsY();

        if (fuelPointsX.length != fuelPointsY.length) {
            // This shouldn't happen but just in case
            return null;
        }

        return doubleArraysToTranslation2dList(fuelPointsX, fuelPointsY);
    }

    /**
     * My first java doc
     * this should return a robot relative Trajectory idk how to implement that
     */
    @Override
    public Trajectory getPath() {
        if (xPointsSubscriberField.exists() && yPointsSubscriberField.exists()) {
            List<Translation2d> translationPoints = getFieldPositions();

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
        inputs.fieldPositions = getFieldPositions();
        inputs.hopperSeesObject = hopperSeesObject();
    }
}