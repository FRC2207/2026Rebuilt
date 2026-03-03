package frc.robot.lib.commands;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.current.FieldConstants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.util.AllianceFlipUtil;

public class AutoAlign extends DriveToPose {
        public static List<Pose2d> reefPositions = new ArrayList<>();
        public static List<Pose2d> sourcePositions = new ArrayList<>();
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("PosArr", Pose2d.struct).publish();
        private static Boolean ReefProtection = false;

        // Update these locations in FIELD CONSTANTS as needed. Don't mess with angles.
        public static final Pose2d hubCenter = new Pose2d(
                        FieldConstants.Elements.blueHub, Rotation2d.fromDegrees(0));
        public static final Pose2d depotCenter = new Pose2d(
                        FieldConstants.Elements.blueDepot, Rotation2d.fromDegrees(90));
        public static final Pose2d outpost = new Pose2d(
                        FieldConstants.Elements.blueOutpost, Rotation2d.fromDegrees(180));
        
        

        public static enum Target {
                SOURCE,
                REEF
        }

        public static enum Direction {
                LEFT,
                RIGHT,
                CENTER
        }

        public AutoAlign(Drive drive, Supplier<Target> target, Supplier<Direction> direction) {
                super(
                        drive,
                         false,
                        () -> {
                                Pose2d nearestTarget;
                                if (target.get() == Target.SOURCE) {
                                        nearestTarget = closestGoal(drive.getPose(), sourcePositions);
                                } else if (target.get() == Target.REEF) {
                                        // if (direction.get() == Direction.CENTER) {
                                        //         nearestTarget = closestGoal(drive.getPose(), reefPositions);
                                        // } else {
                                        //         nearestTarget = offSetReef(closestGoal(drive.getPose(), reefPositions), direction.get());
                                        // }
                                        nearestTarget = offSetReef(drive, closestGoal(drive.getPose(), reefPositions), direction.get());
                                 } else {
                                        nearestTarget = closestGoal(drive.getPose(), reefPositions);
                                }

                                        System.out.print(direction.get());
                                        System.out.print("before flip");

                                        // Dont slam into reef
                                        Pose2d allianceFlippedDrive = AllianceFlipUtil.apply(drive.getPose());

                                // if (ReefProtection) {
                                //         if ((drive.getPose().getY() < FieldConstants.ReefPoints.northPoint.getY() && // If the robot is right of the northmost
                                //                         DriverStation.getAlliance().get() == Alliance.Blue && nearestTarget == driverStationOneSource)||       // point of the BLUE reef
                                //                 (drive.getPose().getY() > AllianceFlipUtil.apply(FieldConstants.ReefPoints.northPoint.getY()) &&                                                      // Or if the robot is right of the
                                //                         DriverStation.getAlliance().get() == Alliance.Red && nearestTarget == driverStationOneSource)) {   // northmost point on RED side
                                                
                                //                         nearestTarget = new Pose2d(nearestTarget.getY(),
                                //                                 MathUtil.clamp(nearestTarget.getX(),
                                //                                                 allianceFlippedDrive.getX(), 2 *
                                //                                                                 (allianceFlippedDrive.getX()
                                //                                                                                 - FieldConstants.ReefPoints.northPoint
                                //                                                                                                 .getX())),
                                //                                 nearestTarget.getRotation());
                                //         } else if ((drive.getPose().getY() > FieldConstants.ReefPoints.southPoint.getY() &&  // If the robot is left of the southmost
                                //                         DriverStation.getAlliance().get() == Alliance.Blue && nearestTarget == driverStationThreeSource) ||               // point of the blue reed
                                //                 (drive.getPose().getY() < AllianceFlipUtil.apply(FieldConstants.ReefPoints.southPoint.getY()) &&                                                   // or if the robot is left of the southmost
                                //                         DriverStation.getAlliance().get() == Alliance.Red && nearestTarget == driverStationThreeSource)) {

                                //                         nearestTarget = new Pose2d(nearestTarget.getY(),
                                //                                 MathUtil.clamp(nearestTarget.getX(),
                                //                                                 allianceFlippedDrive.getX(), 2 *
                                //                                                                 (allianceFlippedDrive.getX()
                                //                                                                                 - FieldConstants.ReefPoints.southPoint
                                //                                                                                                 .getX())),
                                //                                 nearestTarget.getRotation());
                                //         }
                                // }

                                        // // Dont slam into subwoofer
                                        // if ((target.get() == Target.SOURCE || target.get() == Target.REEF) &&
                                        //                 (drive.getPose().getY() < FieldConstants.Speaker.topLeftSpeaker
                                        //                                 .getY() - 0.25
                                        //                                 || drive.getPose()
                                        //                                                 .getY() > FieldConstants.Speaker.topRightSpeaker
                                        //                                                                 .getY() +
                                        //                                                                 0.25)) {
                                        //         nearestTarget = new Pose2d(MathUtil.clamp(nearestTarget.getX(), 1.5,
                                        //                         Double.MAX_VALUE), nearestTarget.getY(),
                                        //                         nearestTarget.getRotation());
                                        // }

                                nearestTarget = AllianceFlipUtil.apply(nearestTarget);
                                System.out.print("after flip");
                                System.out.println(nearestTarget.getRotation());
                                return nearestTarget;
                        }
                );

                // reefPositions.add(northeastReef);
                // reefPositions.add(northwestReef);
                // reefPositions.add(westReef);
                // reefPositions.add(southwestReef);
                // reefPositions.add(southeastReef);
                // reefPositions.add(eastReef);
                double robotLength = Units.inchesToMeters(29.5);
                double reefOffSet = Units.inchesToMeters(6.5);
                double reefRadius = Units.inchesToMeters(32.75);
                double centerOffSet = reefRadius + reefOffSet + (robotLength / 2);

                Pose2d reefOrgin = new Pose2d(Units.inchesToMeters(32.75 + 144), Units.inchesToMeters((26*12)+5)/2, new Rotation2d());

                for (int i = 0; i < 6; i++) {
                        var rotation = reefOrgin.rotateAround(reefOrgin.getTranslation(), Rotation2d.fromDegrees(60 * i));

                        var reefSide = rotation.transformBy(new Transform2d(-centerOffSet, 0.0, new Rotation2d(0)));
                        reefPositions.add(reefSide);
                }
                Pose2d[] arr = new Pose2d[reefPositions.size()];
                arr = reefPositions.toArray(arr);

                publisher.set(arr);
                // Logger.recordOutput("AutoAlign/PosArr", reefPositions.toArray());


                
        }

        public void periodic() {

        }

        /**
         * Method to adjust the position of the bot using preset numbers relative to the REEF.
         * TODO: Test left vs right with new locations for -1's. 19 is backwards
         * @param originalPose The location of the bot without an offset. The reef side you want to go to.
         * @param direction The direction you want the bot to be offseted towards. LEFT, RIGHT, or CENTER
         * @return The new pose that {@link AutoAlign#AutoAlign} will go to. 
         */
        public static Pose2d offSetReef(Drive drive, Pose2d originalPose, Direction direction) {
                Pose2d updatedPose;

                switch (direction) {
                        case LEFT:
                                /* updatedPose = originalPose.transformBy(
                                                new Transform2d(-1 * Math.sin(originalPose.getRotation().getRadians() + Math.PI / 2) * (FieldConstants.postDistance / 2),
                                                                Math.cos(originalPose.getRotation().getRadians() + Math.PI / 2) * (FieldConstants.postDistance / 2),
                                                                new Rotation2d(0)));*/
                                updatedPose = originalPose.transformBy(
                                        new Transform2d(0,
                                                        (12 / 2),
                                                        new Rotation2d(0)));
                                break;
                        case RIGHT:
                                /* updatedPose = originalPose.plus(
                                                new Transform2d(Math.sin(originalPose.getRotation().getRadians() + Math.PI / 2) * (FieldConstants.postDistance / 2),
                                                                -1 * Math.cos(originalPose.getRotation().getRadians() + Math.PI / 2) * (FieldConstants.postDistance / 2),
                                                                new Rotation2d(0))); */
                                updatedPose = originalPose.transformBy(
                                        new Transform2d(0,
                                                        -1 * (12 / 2),
                                                        new Rotation2d(0)));
                                break;
                        case CENTER:
                                updatedPose = originalPose;
                                break;
                        default:
                                updatedPose = originalPose;
                }

                return updatedPose;
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
}
