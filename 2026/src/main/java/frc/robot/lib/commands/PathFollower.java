package frc.robot.lib.commands;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.current.FieldConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;
import frc.robot.lib.util.AllianceRotationUtil;

public class PathFollower extends Command {
        public boolean running = false;
        public boolean inside;
        private Drive drive;

        public static List<Pose2d> trenchPositions = new ArrayList<>();

        private final LoggedDashboardChooser<TrenchOptions> m_chooser = new LoggedDashboardChooser<>(
                        "PathFollower/Chooser");

        private Pose2d goalPosition;
        private TrenchOptions selected;
        private Target target;

        private static PathConstraints constraints;
        private Command pathFindingCommand;

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

        public static enum TrenchOptions {
                NEAREST,
                CLOCKWISE,
                COUNTERCLOCKWISE,
                FORCELEFT,
                FORCERIGHT
        }

        public PathFollower(Drive drive, Target target) {
                this.drive = drive;
                this.target = target;

                trenchPositions.add(leftTrench);
                trenchPositions.add(rightTrench);

                m_chooser.addDefaultOption("Nearest", TrenchOptions.NEAREST);
                m_chooser.addOption("Clockwise", TrenchOptions.CLOCKWISE);
                m_chooser.addOption("Counterclockwise", TrenchOptions.COUNTERCLOCKWISE);
                m_chooser.addOption("Force Left", TrenchOptions.FORCELEFT);
                m_chooser.addOption("Force Right", TrenchOptions.FORCERIGHT);

                constraints = new PathConstraints(
                                DriveConstants.maxSpeedMetersPerSec * .75, 3.0,
                                Math.PI * 2, Units.degreesToRadians(720));

                addRequirements(drive);
                // CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        }

        @Override
        public void initialize() {
                // Basic boolean which can be used in other subsystems.
                running = true;

                Pose2d whichTrenchOut;
                Pose2d whichTrenchIn;

                selected = m_chooser.get();

                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                        if (FieldConstants.fieldLength - FieldConstants.neutralLine < drive.getPose().getX()) {
                                inside = true;
                        } else {
                                inside = false;
                        }
                } else {
                        if (drive.getPose().getX() < FieldConstants.neutralLine) {
                                inside = true;
                        } else {
                                inside = false;
                        }
                }

                // Determine which trench to go to based on the selected option from
                // Smartdashboard
                if (target == Target.TRENCH) {
                        switch (selected) {
                                case CLOCKWISE:
                                        whichTrenchOut = leftTrench;
                                        whichTrenchIn = rightTrench;
                                        break;
                                case COUNTERCLOCKWISE:
                                        whichTrenchOut = rightTrench;
                                        whichTrenchIn = leftTrench;
                                        break;
                                case FORCELEFT:
                                        whichTrenchOut = leftTrench;
                                        whichTrenchIn = leftTrench;
                                        break;
                                case FORCERIGHT:
                                        whichTrenchOut = rightTrench;
                                        whichTrenchIn = rightTrench;
                                        break;
                                case NEAREST:
                                        whichTrenchOut = closestGoal(AllianceRotationUtil.apply(drive.getPose()),
                                                        trenchPositions);
                                        whichTrenchIn = closestGoal(AllianceRotationUtil.apply(drive.getPose()),
                                                        trenchPositions);
                                        break;
                                default:
                                        whichTrenchOut = closestGoal(AllianceRotationUtil.apply(drive.getPose()),
                                                        trenchPositions);
                                        whichTrenchIn = closestGoal(AllianceRotationUtil.apply(drive.getPose()),
                                                        trenchPositions);
                                        Commands.print("Invalid trench option selected, defaulting to nearest");
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
                }

                Logger.recordOutput("PathFollower/PreflipGoalPosition", goalPosition);
                // Takes the previous position and applies alliance rotation if need.
                goalPosition = AllianceRotationUtil.apply(goalPosition);

                // Record the goal position and selected trench option to the logger for
                // debugging purposes
                Logger.recordOutput("PathFollower/GoalPosition", goalPosition);
                Logger.recordOutput("PathFollower/SelectedTrenchOption", selected);

                // Builds the path using the position we just finalized
                pathFindingCommand = AutoBuilder.pathfindToPose(
                                goalPosition,
                                constraints,
                                0.0);

                pathFindingCommand.initialize();
        }

        @Override
        public void execute() {
                Logger.recordOutput("PathFollower/SelectedTrenchOption", selected);

                pathFindingCommand.execute();
                // Schedules the command. This is what runs the path.
                //CommandScheduler.getInstance().schedule(pathFindingCommand);
        }

        @Override
        public void end(boolean interrupted) {
                running = false;
                drive.stop();
                pathFindingCommand.end(interrupted);
        }

        @Override
        public boolean isFinished() {
                return pathFindingCommand.isFinished();
        }

        /**
         * Returns whether the PathFollower is currently running. Useful for other
         * commands
         * 
         * @return a running boolean
         */
        public boolean isRunning() {
                return running;
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
                        double distance = calculateDistance(currentPose, position);

                        if (distance < shortestDistance) {
                                shortestDistance = distance;
                                closestGoal = position;
                        }
                }

                return closestGoal;
        }

        /**
         * Calculates the hypotenuse distance between any Pose2d's.
         * 
         * @param p1 the first point
         * @param p2 the second point
         * @return The distance between the two points.
         */
        private static double calculateDistance(Pose2d p1, Pose2d p2) {
                return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
        }
}
