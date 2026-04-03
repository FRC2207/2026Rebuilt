package frc.robot.current;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;
import frc.robot.lib.commands.PathFollower;
import frc.robot.lib.util.AllianceRotationUtil;

public class Pather {

    public boolean running = false;
    public static boolean inside;
    public static boolean isPathing;

    // keep list static but populate only once
    public static List<Pose2d> trenchPositions = new ArrayList<>();

    private static PathConstraints constraints;

    // Update these locations in FIELD CONSTANTS as needed. Don't mess with angles.
    public static final Pose2d hubCenter = new Pose2d(
            FieldConstants.Elements.blueHub, Rotation2d.fromDegrees(0));
    public static final Pose2d hubShoot = new Pose2d(
            FieldConstants.Elements.blueHubShoot, Rotation2d.fromDegrees(0));
    public static final Pose2d depotCenter = new Pose2d(
            FieldConstants.Elements.blueDepot, Rotation2d.fromDegrees(90));
    public static final Pose2d outpost = new Pose2d(
            FieldConstants.Elements.blueOutpost, Rotation2d.fromDegrees(0));
    public static final Pose2d leftTrench = new Pose2d(
            FieldConstants.Elements.leftTrench, Rotation2d.fromDegrees(0));
    public static final Pose2d rightTrench = new Pose2d(
            FieldConstants.Elements.rightTrench, Rotation2d.fromDegrees(0));

    public static enum Target {
        TRENCH,
        OUTPOST,
        HUBSHOOTCENTER,
        HUBSHOOTLEFT,
        HUBSHOOTRIGHT
    }

    public static enum Direction {
        LEFT,
        RIGHT,
        CENTER
    }

    public static enum TrenchOptions {
        CLOCKWISE,
        COUNTERCLOCKWISE,
        FORCELEFT,
        FORCERIGHT,
        NEAREST
    }

    // static initializer: populate trenchPositions and configure constraints
    // exactly once
    static {
        trenchPositions.add(leftTrench);
        trenchPositions.add(rightTrench);

        constraints = new PathConstraints(
                DriveConstants.maxSpeedMetersPerSec * .75, 3.0,
                Math.PI * 2, Units.degreesToRadians(720));
    }

    /**
     * Binds certain methods to network table entries
     */
    public static void configureKindleListeners() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("PresetTriggers");

        Trigger trenchLTgr = new Trigger(() -> table.getEntry("Trench Left").getBoolean(false));
        trenchLTgr.whileTrue(Commands.deferredProxy(() -> trenchAlign(Direction.LEFT)));

        Trigger trenchRTgr = new Trigger(() -> table.getEntry("Trench Right").getBoolean(false));
        trenchRTgr.whileTrue(Commands.deferredProxy(() -> trenchAlign(Direction.RIGHT)));

        Trigger shootLTgr = new Trigger(() -> table.getEntry("ShootL").getBoolean(false));
        shootLTgr.whileTrue(Commands.deferredProxy(() -> pathFinderPro(Target.HUBSHOOTLEFT)));

        Trigger shootMTgr = new Trigger(() -> table.getEntry("ShootM").getBoolean(false));
        shootMTgr.whileTrue(Commands.deferredProxy(() -> pathFinderPro(Target.HUBSHOOTCENTER)));

        Trigger shootRTgr = new Trigger(() -> table.getEntry("ShootR").getBoolean(false));
        shootRTgr.whileTrue(Commands.deferredProxy(() -> pathFinderPro(Target.HUBSHOOTRIGHT)));

        Trigger outpostTgr = new Trigger(() -> table.getEntry("Outpost").getBoolean(false));
        outpostTgr.whileTrue(Commands.deferredProxy(() -> pathFinderPro(Target.OUTPOST)));

        SmartDashboard.putBoolean("ShootL", table.getEntry("ShootL").getBoolean(false));

        SmartDashboard.putData("ShootR", Commands.deferredProxy(() -> pathFinderPro(Target.HUBSHOOTRIGHT)));
    }

    public static Command pathFinder(Target target) {
        Pose2d goalPosition;

        switch (target) {
            case TRENCH:
                goalPosition = closestGoal(AutoBuilder.getCurrentPose(), trenchPositions);
                break;
            case OUTPOST:
                goalPosition = outpost;
                break;
            case HUBSHOOTCENTER:
                goalPosition = new Pose2d(hubCenter.getTranslation().plus(new Translation2d(-2.825, 0)),
                        new Rotation2d(Units.degreesToRadians(90)));
                break;
            case HUBSHOOTLEFT:
                goalPosition = new Pose2d(hubCenter.getTranslation().plus(new Translation2d(-2.62, 2)),
                        new Rotation2d(Units.degreesToRadians(90) - .7));
                break;
            case HUBSHOOTRIGHT:
                goalPosition = new Pose2d(hubCenter.getTranslation().plus(new Translation2d(-2.62, -2)),
                        new Rotation2d(Units.degreesToRadians(90) + .7));
                break;
            default:
                goalPosition = AutoBuilder.getCurrentPose();
        }

        Logger.recordOutput("PathFollower/PreflipGoalPosition", goalPosition);
        // Takes the previous position and applies alliance rotation if need.
        // goalPosition = AllianceRotationUtil.apply(goalPosition);

        // Record the goal position and selected trench option to the logger for
        // debugging purposes
        Logger.recordOutput("PathFollower/GoalPosition", goalPosition);

        return pathFinder(goalPosition, false).finallyDo(() -> isPathing = false);
        
    }

    /**
     * Finds a path to a pose2d from anywhere on the field.
     * 
     * @param targetPose    the target pose to pathfind to
     * @param forceAlliance if true, the pathfinder will not apply alliance rotation
     *                      to the target
     * @return
     */
    public static Command pathFinder(Pose2d targetPose, Boolean forceAlliance) {
        Pose2d goalPosition;

        if (!forceAlliance) {
            goalPosition = targetPose;
            Logger.recordOutput("PathFollower/PreflipGoalPosition", goalPosition);
            goalPosition = AllianceRotationUtil.apply(goalPosition);
            Logger.recordOutput("PathFollower/GoalPosition", goalPosition);
        } else {
            goalPosition = targetPose;
            Logger.recordOutput("PathFollower/GoalPosition", goalPosition);
        }

        return AutoBuilder.pathfindToPose(
                goalPosition,
                constraints,
                0.0);
    }

    public static Command pathFinderPro(Target target) {
        isPathing = true;
        if (isInside()) {
            return pathFinder(target);
        } else {
            if (target == Target.HUBSHOOTLEFT) {
                return Commands.sequence(trenchAlign(Direction.LEFT),
                        pathFinder(target));
            } else if (target == Target.HUBSHOOTRIGHT) {
                return Commands.sequence(trenchAlign(Direction.RIGHT),
                        pathFinder(target));
            } else {
                return Commands.sequence(trenchAlign(Direction.RIGHT),
                        pathFinder(target));
            }
        }
    }

    public static Command trenchAlign(Direction direction) {
        isPathing = true;
        // determine inside once
        double currentX = AutoBuilder.getCurrentPose().getX();
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            inside = FieldConstants.fieldLength - FieldConstants.neutralLine < currentX;
        } else {
            inside = currentX < FieldConstants.neutralLine;
        }

        // select file name based on direction + inside/outside
        final String filename;
        switch (direction) {
            case LEFT:
                filename = inside ? "Left Trench Out" : "Left Trench In";
                break;
            case RIGHT:
                filename = inside ? "Right Trench Out" : "Right Trench In";
                break;
            default:
                filename = "Left Trench Out";
                break;
        }

        // attempt to build the path-follow command once
        try {
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(filename), constraints).finallyDo(() -> isPathing = false);
        } catch (FileVersionException | IOException | ParseException e) {
            // Log and return a no-op command on error
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static Command climbAlign(Direction direction) {
        // select file name based on direction + inside/outside
        final String filename;
        switch (direction) {
            case LEFT:
                filename = "Left Climb";
                break;
            case RIGHT:
                filename = "Right Climb";
                break;
            case CENTER:
                filename = "Center Climb";
                break;
            default:
                filename = "Center Climb";
                break;
        }
        // attempt to build the path-follow command once
        try {
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(filename), constraints);
        } catch (FileVersionException | IOException | ParseException e) {
            // Log and return a no-op command on error
            e.printStackTrace();
            return Commands.none();
        }
    }

    /**
     * Assesses all reef locations and determines which one is the closest to the
     * current position
     * 
     * @param currentPose is where the robot is currently
     * @param positions   is which list of positions you want to assess
     * @return which pose2d the {@link PathFollower#AutoAlign} will travel to
     */
    private static Pose2d closestGoal(Pose2d currentPose, List<Pose2d> positions) {
        Pose2d closestGoal = null;
        double shortestDistance = Double.MAX_VALUE;

        for (Pose2d position : positions) {
            double distance = currentPose.getTranslation().getDistance(position.getTranslation());

            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestGoal = position;
            }
        }

        return closestGoal;
    }

    private static boolean isInside() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            if (FieldConstants.fieldLength - FieldConstants.neutralLine < AutoBuilder.getCurrentPose().getX()) {
                inside = true;
            } else {
                inside = false;
            }
        } else {
            if (AutoBuilder.getCurrentPose().getX() < FieldConstants.neutralLine) {
                inside = true;
            } else {
                inside = false;
            }
        }

        return inside;

    }
}
