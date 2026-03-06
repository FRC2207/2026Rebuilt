package frc.robot.current.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.current.FieldConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;
import frc.robot.lib.commands.DriveToPose;
import frc.robot.lib.util.AllianceFlipUtil;

public class AutoAlign extends DriveToPose {
        private Drive drive;
        public static List<Pose2d> trenchPositions = new ArrayList<>();
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("PosArr", Pose2d.struct).publish();
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

        public AutoAlign(Drive drive, Supplier<Target> target) {
                super(
                                drive,
                                false,
                                () -> {
                                        Pose2d nearestTarget = drive.getPose();

                                        trenchPositions.add(leftTrench);
                                        trenchPositions.add(rightTrench);

                                        if (target.get() == Target.OUTPOST) {
                                                nearestTarget = outpost;
                                        } else if (target.get() == Target.TRENCH) {
                                                nearestTarget = closestGoal(drive.getPose(), trenchPositions);
                                        } else if (target.get() == Target.HUB) {
                                                nearestTarget = hubCenter;
                                        }

                                        Pose2d allianceFlippedDrive = AllianceFlipUtil.apply(drive.getPose());

                                        System.out.print("after flip");
                                        System.out.println(nearestTarget.getRotation());
                                        return nearestTarget;
                                });

                this.drive = drive;
                constraints = new PathConstraints(
                                DriveConstants.maxSpeedMetersPerSec, 4.0,
                                Math.PI * 2, Units.degreesToRadians(720));
        }

        public void periodic() {

        }

        /**
         * Assesses all reef locations and determines which one is the closest to the
         * current position
         * 
         * @param currentPose is where the robot is currently
         * @param positions   is which list of positions you want to assess
         * @return which pose2d the {@link AutoAlign#AutoAlign} will travel to
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

        public Command driveThruTrench() {
                Pose2d goalTarget = closestGoal(drive.getPose(), trenchPositions);
                
                Command pathfindingCommand = AutoBuilder.pathfindToPose(
                                goalTarget,
                                constraints,
                                0.0);

                return pathfindingCommand;
        }
}
