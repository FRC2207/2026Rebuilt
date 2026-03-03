package frc.robot.current.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.current.FieldConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;

public class PathFollower {
        private Drive drive;
        public static List<Pose2d> trenchPositions = new ArrayList<>();
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("PosArr", Pose2d.struct).publish();

        private final SendableChooser<TrenchOptions> m_chooser = new SendableChooser<>();


        private static PathConstraints constraints;

        private static TrenchOptions trenchOptions;

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

        public static enum TrenchOptions {
                NEAREST,
                CLOCKWISE,
                COUNTERCLOCKWISE,
                FORCELEFT,
                FORCERIGHT
        }

        public PathFollower(Drive drive) {
                this.drive = drive;
                //Pose2d allianceFlippedDrive = AllianceFlipUtil.apply(drive.getPose());

                trenchPositions.add(leftTrench);
                trenchPositions.add(rightTrench);

                m_chooser.setDefaultOption("Nearest", trenchOptions = TrenchOptions.NEAREST);
                m_chooser.addOption("Clockwise", trenchOptions = TrenchOptions.CLOCKWISE);
                m_chooser.addOption("Counterclockwise", trenchOptions = TrenchOptions.COUNTERCLOCKWISE);
                m_chooser.addOption("Force Left", trenchOptions = TrenchOptions.FORCELEFT);
                m_chooser.addOption("Force Right", trenchOptions = TrenchOptions.FORCERIGHT);
                
                SmartDashboard.putData(m_chooser);
                
                // Configure AutoBuilder for PathPlanner. This might not be necessary here. Also
                // in Drive subsystem.
                // AutoBuilder.configure(
                //                 drive::getPose,
                //                 drive::setPose,
                //                 drive::getChassisSpeeds,
                //                 drive::runVelocity,
                //                 new PPHolonomicDriveController(
                //                                 new PIDConstants(DriveConstants.driveKp, DriveConstants.driveKi,
                //                                                 DriveConstants.driveKd),
                //                                 new PIDConstants(DriveConstants.turnKp, DriveConstants.turnKi,
                //                                                 DriveConstants.turnKd)),
                //                 DriveConstants.ppConfig,
                //                 () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                //                 drive);
                // Pathfinding.setPathfinder(new LocalADStarAK());

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

        /** Returns a command to drive to the outpost  */
        public Command driveToOutpost() {
                Command pathfindingCommand = AutoBuilder.pathfindToPose(
                                FieldConstants.Elements.blueOutpostPose,
                                constraints,
                                0.0);

                return pathfindingCommand;
        }
        /**
         * Returns a command to drive through the trench. 
         * If we are outside, it will drive through to the alliance
         * area, and if we are inside, it will drive through to the outside.
         */
        public Command driveThruTrench() {
                Pose2d whichTrenchOut;
                Pose2d whichTrenchIn;
                Pose2d goalPosition;

                switch (trenchOptions) {
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
                        case NEAREST:
                        default:
                                whichTrenchOut = closestGoal(drive.getPose(), trenchPositions);
                                whichTrenchIn = closestGoal(drive.getPose(), trenchPositions);
                                break;
                }

                if (drive.getPose().getX() < FieldConstants.neutralLine) { // If we are inside
                        goalPosition = new Pose2d(
                                        whichTrenchOut.getTranslation()
                                                        .plus(new Translation2d(Units.inchesToMeters(55), 0)),
                                        whichTrenchOut.getRotation());
                } else { // If we are outside
                        goalPosition = new Pose2d(
                                        whichTrenchIn.getTranslation()
                                                        .minus(new Translation2d(Units.inchesToMeters(22), 0)),
                                        whichTrenchIn.getRotation());
                }

                Commands.print("Goal position:" + goalPosition);
                
                Command pathfindingCommand = AutoBuilder.pathfindToPose(
                                goalPosition,
                                constraints,
                                0.0);

                return pathfindingCommand;
        }
}
