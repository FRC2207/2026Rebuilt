package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.subsystems.swerveDrive.Drive;

public class ObjectVision extends SubsystemBase {
    private Drive swerve;

    private PathConstraints constraints;
    private double recalculatePathPeriod = 0.5;
    private ObjectVisionIO io;
    private boolean debugFuelPoints = false;
    private final ObjectVisionIOInputsAutoLogged inputs = new ObjectVisionIOInputsAutoLogged();

    public ObjectVision(Drive drive, ObjectVisionIO io) {
        this.io = io;
        this.swerve = drive;
        
        constraints = new PathConstraints(
            swerve.getMaxLinearSpeedMetersPerSec(), // Max velocity
            2.0, // Max acceleration hopefully 2 mps^2 is safe
            swerve.getMaxAngularSpeedRadPerSec(), // Max rotation velocity
            Units.degreesToRadians(720) // Max rotation acceleration, this is probaly okay
        );
    }

    private double[] getFuelPointsX() {
        return inputs.fuelX;
    }

    private double[] getFuelPointsY() {
        return inputs.fuelY;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("ObjectVision", inputs);

        if (debugFuelPoints) {
            List<Translation2d> points = getObjectPostionsRelativeToRobot();
            if (points != null) {
                Pose2d robotPose = swerve.getPose();
                Pose2d[] fieldPoses = points.stream()
                    .map(p -> robotPose.transformBy(new Transform2d(p, new Rotation2d())))
                    .toArray(Pose2d[]::new);
                Logger.recordOutput("ObjectVision/FuelPoints", fieldPoses);
            }
        }
    }

    private List<Translation2d> doubleArraysToTranslation2dList(double[] xArray, double[] yArray) {
        List<Translation2d> poses = new ArrayList<>();
        int length = Math.min(xArray.length, yArray.length);
        for (int i = 0; i < length; i++) {
            poses.add(new Translation2d(xArray[i], yArray[i]));
        }
        return poses;
    }

    public Boolean hopperSeesObject() {
        return inputs.hopperSeesObject;
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
    public Command getPath() {
        List<Translation2d> relativePoints = getObjectPostionsRelativeToRobot();
        if (relativePoints == null || relativePoints.isEmpty()) return Commands.none();

        Pose2d robotPose = swerve.getPose();
        List<Pose2d> poses = new ArrayList<>();
        
        poses.add(robotPose);

        // Convert to field relative points (which is what PathPlannerPath works with)
        for (Translation2d relToken : relativePoints) {
            Pose2d fieldPose = robotPose.transformBy(new Transform2d(relToken, new Rotation2d()));
            poses.add(fieldPose);
        }

        // Create waypoints which will help the path generation
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, robotPose.getRotation()) // Stop at the end, maybe not ideal can fix later
        );

        path.preventFlipping = true;

        // Plotting for AdvantageScope debugging
        // inst.getTable("Vision").getStructArrayTopic("PlannedPathHailMary", Pose2d.struct)
            // .publish(path.getPathPoses().toArray(new Pose2d[0]));

        return AutoBuilder.followPath(path);
    }

    public void setDebugFuelPoints(boolean debug) {
        this.debugFuelPoints = debug;
    }


    public Command getDynamicPath() {
        return Commands.deferredProxy(() -> {
            Command followCommand = getPath(); 
            
            if (followCommand == null) return Commands.waitSeconds(0.1);

            return followCommand.withTimeout(recalculatePathPeriod); 
        })
        .repeatedly()
        .finallyDo(() -> swerve.runVelocity(new ChassisSpeeds())); // Emergency stop
    }
}