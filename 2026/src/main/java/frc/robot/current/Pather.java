package frc.robot.current;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;
import frc.robot.lib.commands.PathFollower;
import frc.robot.lib.util.AllianceRotationUtil;

public class Pather {

    public boolean running = false;
    public static boolean inside;

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
        HUBSHOOT
    }

    public static enum Direction {
        LEFT,
        RIGHT
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

    public static Command pathFinder(Target target) {
        return pathFinder(target, null);
    }

    public static Command pathFinder(Target target, Supplier<TrenchOptions> trenchOptionsSupplier) {
        // Guard against a null supplier BEFORE calling get()
        TrenchOptions selected = TrenchOptions.NEAREST;
        if (trenchOptionsSupplier != null) {
            TrenchOptions opt = trenchOptionsSupplier.get();
            if (opt != null) {
                selected = opt;
            }
        }

        Pose2d whichTrenchOut;
        Pose2d whichTrenchIn;
        Pose2d goalPosition;

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

        // Determine which trench to go to based on the selected option
        if (target == Target.TRENCH) {
            switch (selected) {
                case CLOCKWISE:
                    whichTrenchOut = leftTrench;
                    whichTrenchIn = rightTrench;
                    Logger.recordOutput("Pather/selected", selected);
                    break;
                case COUNTERCLOCKWISE:
                    whichTrenchOut = rightTrench;
                    whichTrenchIn = leftTrench;
                    Logger.recordOutput("Pather/selected", selected);
                    break;
                case FORCELEFT:
                    whichTrenchOut = leftTrench;
                    whichTrenchIn = leftTrench;
                    Logger.recordOutput("Pather/selected", selected);
                    break;
                case FORCERIGHT:
                    whichTrenchOut = rightTrench;
                    whichTrenchIn = rightTrench;
                    Logger.recordOutput("Pather/selected", selected);
                    break;
                case NEAREST:
                    whichTrenchOut = closestGoal(AllianceRotationUtil.apply(AutoBuilder.getCurrentPose()),
                            trenchPositions);
                    whichTrenchIn = closestGoal(AllianceRotationUtil.apply(AutoBuilder.getCurrentPose()),
                            trenchPositions);
                    break;
                default:
                    whichTrenchOut = closestGoal(AllianceRotationUtil.apply(AutoBuilder.getCurrentPose()),
                            trenchPositions);
                    whichTrenchIn = closestGoal(AllianceRotationUtil.apply(AutoBuilder.getCurrentPose()),
                            trenchPositions);
                    Commands.print("Invalid trench option selected, defaulting to nearest");
                    SmartDashboard.putBoolean("Running default", true);
                    break;
            }

            // Uses the current position of the robot to determine whether to go to the
            // inside or outside trench position.
            if (inside) { // If we are inside
                goalPosition = new Pose2d(
                        whichTrenchOut.getTranslation()
                                .plus(new Translation2d(Units.inchesToMeters(55), 0)),
                        whichTrenchOut.getRotation());
            } else { // If we are outside
                goalPosition = new Pose2d(
                        whichTrenchIn.getTranslation()
                                .minus(new Translation2d(Units.inchesToMeters(55), 0)),
                        whichTrenchIn.getRotation());
            }
            // The other options to pathFind to.
        } else if (target == Target.OUTPOST) {
            goalPosition = outpost;
        } else if (target == Target.HUBSHOOT) {
            goalPosition = new Pose2d(hubCenter.getTranslation().plus(new Translation2d(-2.825, 0)),
                    new Rotation2d(Units.degreesToRadians(90)));
        } else {
            goalPosition = AutoBuilder.getCurrentPose();
        }

        Logger.recordOutput("PathFollower/PreflipGoalPosition", goalPosition);
        // Takes the previous position and applies alliance rotation if need.
        goalPosition = AllianceRotationUtil.apply(goalPosition);

        // Record the goal position and selected trench option to the logger for
        // debugging purposes
        Logger.recordOutput("PathFollower/GoalPosition", goalPosition);
        if (trenchOptionsSupplier != null) {
            Logger.recordOutput("PathFollower/SelectedTrenchOption", trenchOptionsSupplier.get());
        }

        return pathFinder(goalPosition, true);
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

    public static Command trenchAlign(Direction direction) {
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
}
