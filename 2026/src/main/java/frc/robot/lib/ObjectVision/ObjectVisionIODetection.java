package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.current.subsystems.swerveDrive.Drive;

public class ObjectVisionIODetection implements ObjectVisionIO {
    private Drive swerve;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("YellowDetectorFieldData");
    private DoubleArraySubscriber xPointsSubscriberField;
    private DoubleArraySubscriber yPointsSubscriberField;

    private BooleanSubscriber hopperSubscriber;
    private PathConstraints constraints;
    private double recalculatePathPeriod = 0.5;

    public ObjectVisionIODetection(Drive drive) {
        this.swerve = drive;
        
        constraints = new PathConstraints(
            swerve.getMaxLinearSpeedMetersPerSec(), // Max velocity
            2.0, // Max acceleration hopefully 2 mps^2 is safe
            swerve.getMaxAngularSpeedRadPerSec(), // Max rotation velocity
            Units.degreesToRadians(720) // Max rotation acceleration, this is probaly okay
        );

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

    private List<Translation2d> getObjectPostionsRelativeToRobot() {
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
    public Command getPath() {
        if (xPointsSubscriberField.exists() && yPointsSubscriberField.exists()) {
            List<Translation2d> relativePoints = getObjectPostionsRelativeToRobot();

            List<Pose2d> poses = new ArrayList<>();
            poses.add(new Pose2d(0, 0, new Rotation2d()));

            for (Translation2d pt : relativePoints) {
                poses.add(new Pose2d(pt, new Rotation2d())); 
            }

            double currentSpeed = swerve.getLinearVelocity();

            // If almost stopped, use like a small kick start at 0.5 mps^2
            double targetEndSpeed = Math.max(currentSpeed, 0.5); 

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // starting rotation, null will set it to current
                new GoalEndState(targetEndSpeed, swerve.getRotation()) // This means it tries to end at the current speed goo
            );

            path.preventFlipping = true; // smth to do with alliance color, this should be correct
            return AutoBuilder.followPath(path);
        } else {
            return null;
        }
    }

    @Override
    public Command getDynamicPath() {
        return Commands.deferredProxy(() -> {
            Command followCommand = getPath(); 
            
            if (followCommand == null) return Commands.waitSeconds(0.1);

            return followCommand.withTimeout(recalculatePathPeriod); 
        })
        .repeatedly()
        .finallyDo(() -> swerve.runVelocity(new ChassisSpeeds())); // Emergency stop
    }

    @Override
    public void updateInputs(objectVisionIOInputs inputs) {
        inputs.fieldPositions = getObjectPostionsRelativeToRobot();
        inputs.hopperSeesObject = hopperSeesObject();
    }
}