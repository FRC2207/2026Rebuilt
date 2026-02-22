package frc.robot.current.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.current.FieldConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;
import frc.robot.lib.commands.DriveToPose;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.LocalADStarAK;

public class PathFollower {
        private Drive drive;
        public static List<Pose2d> trenchPositions = new ArrayList<>();
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("PosArr", Pose2d.struct).publish();
        private static Boolean ReefProtection = false;

        private static PathConstraints constraints;

        // Update these locations in FIELD CONSTANTS as needed. Don't mess with angles.
        public static final Pose2d hubCenter = new Pose2d(
                        FieldConstants.Elements.blueHub, Rotation2d.fromDegrees(0));
        public static final Pose2d depotCenter = new Pose2d(
                        FieldConstants.Elements.blueDepot, Rotation2d.fromDegrees(90));
        public static final Pose2d outpost = new Pose2d(
                        FieldConstants.Elements.blueOutpost, Rotation2d.fromDegrees(180));
        public static final Pose2d leftTrench = new Pose2d(
                        FieldConstants.Elements.leftTrench, Rotation2d.fromDegrees(180));
        public static final Pose2d rightTrench = new Pose2d(
                        FieldConstants.Elements.rightTrench, Rotation2d.fromDegrees(180));

        public static enum Target {
                TRENCH,
                OUTPOST,
                HUB
        }

        public static enum Direction {
                LEFT,
                RIGHT
        }

        public PathFollower(Drive drive) {
                this.drive = drive;
                Pose2d allianceFlippedDrive = AllianceFlipUtil.apply(drive.getPose());

                // Configure AutoBuilder for PathPlanner. This might not be necessary here. Also in Drive subsystem.
                AutoBuilder.configure(
                                drive::getPose,
                                drive::setPose,
                                drive::getChassisSpeeds,
                                drive::runVelocity,
                                new PPHolonomicDriveController(
                                                new PIDConstants(DriveConstants.driveKp, DriveConstants.driveKi,
                                                                DriveConstants.driveKd),
                                                new PIDConstants(DriveConstants.turnKp, DriveConstants.turnKi,
                                                                DriveConstants.turnKd)),
                                DriveConstants.ppConfig,
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                drive);
                Pathfinding.setPathfinder(new LocalADStarAK());


                constraints = new PathConstraints(
                                DriveConstants.maxSpeedMetersPerSec, 4.0,
                                Math.PI * 2, Units.degreesToRadians(720));

                PathfindingCommand.warmupCommand().schedule(); // Helps remove delay when running the first path.
        }

        public void periodic() {

        }

        /**
         * Assesses all reef locations and determines which one is the closest to the
         * current position
         * 
         * @param currentPose is where the robot is currently
         * @param positions   is which list of positions you want to assess
         * @return which pose2d the {@link PathFollower#AutoAlign} will travel to
         */
        public static Pose2d closestGoal(Pose2d currentPose, List<Pose2d> positions) {
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

        /**
         * Returns a command to drive through the trench. This currently stops directly
         * under the trench.
         * TODO: Expand this so if we are outside, it will drive through to the alliance
         * area, and if we are inside, it will drive through to the outside.
         */
        public Command driveThruTrench() {
                Pose2d whichTrench = closestGoal(drive.getPose(), trenchPositions);
                Pose2d goalPosition;

                if (drive.getPose().getX() < FieldConstants.neutralLine) {      // If we are inside
                        goalPosition = new Pose2d(
                                        whichTrench.getTranslation().plus(new Translation2d(1.5, 0)),
                                        whichTrench.getRotation());
                } else {                                                        // If we are outside
                        goalPosition = new Pose2d(
                                        whichTrench.getTranslation().minus(new Translation2d(1, 0)),
                                        whichTrench.getRotation());
                }

                Command pathfindingCommand = AutoBuilder.pathfindToPose(
                                goalPosition,
                                constraints,
                                0.0);

                return pathfindingCommand;
        }
}
