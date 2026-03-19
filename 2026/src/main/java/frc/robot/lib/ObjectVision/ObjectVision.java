package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.subsystems.swerveDrive.Drive;
import edu.wpi.first.math.util.Units;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;

public class ObjectVision extends SubsystemBase {
    private final Drive swerve;
    private final ObjectVisionIO io;
    private final ObjectVisionIOInputsAutoLogged inputs = new ObjectVisionIOInputsAutoLogged();

    private static final double MIN_BALL_DISTANCE_M = 0.3;
    private static final double RECALCULATE_PERIOD_S = 0.4;
    
    private static final double DBSCAN_EPS = 1.2;
    private static final int DBSCAN_MIN_PTS = 2;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
                                DriveConstants.maxSpeedMetersPerSec, 3.0,
                                Math.PI * 2, Units.degreesToRadians(720));

    private static final int MAX_BALLS = 100;
    private final Pose3d[] ballPosesBuf = new Pose3d[MAX_BALLS];

    public ObjectVision(Drive drive, ObjectVisionIO io) {
        this.io     = io;
        this.swerve = drive;
        for (int i = 0; i < MAX_BALLS; i++) ballPosesBuf[i] = new Pose3d();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectVision", inputs);

        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null) { Logger.recordOutput("ObjectVision/Balls", new Pose3d[0]); return; }
        int n = Math.min(rx.length, MAX_BALLS);
        for (int i = 0; i < n; i++)
            ballPosesBuf[i] = new Pose3d(rx[i], ry[i], 0.0, new Rotation3d());
        Logger.recordOutput("ObjectVision/Balls", java.util.Arrays.copyOf(ballPosesBuf, n));
    }

    private static List<List<Translation2d>> dbscan(List<Translation2d> points) {
        List<List<Translation2d>> clusters = new ArrayList<>();
        Set<Translation2d> visited = new HashSet<>();
        Set<Translation2d> noise = new HashSet<>();

        for (Translation2d point : points) {
            if (visited.contains(point)) continue;
            visited.add(point);

            List<Translation2d> neighbors = getNeighbors(point, points);

            if (neighbors.size() < DBSCAN_MIN_PTS) {
                noise.add(point);
            } else {
                List<Translation2d> cluster = new ArrayList<>();
                expandCluster(point, neighbors, cluster, visited, points);
                clusters.add(cluster);
            }
        }
        return clusters;
    }

    private static void expandCluster(Translation2d point, List<Translation2d> neighbors, List<Translation2d> cluster, Set<Translation2d> visited, List<Translation2d> allPoints) {
        cluster.add(point);
        for (int i = 0; i < neighbors.size(); i++) {
            Translation2d neighbor = neighbors.get(i);
            if (!visited.contains(neighbor)) {
                visited.add(neighbor);
                List<Translation2d> nextNeighbors = getNeighbors(neighbor, allPoints);
                if (nextNeighbors.size() >= DBSCAN_MIN_PTS) {
                    neighbors.addAll(nextNeighbors);
                }
            }
            if (!cluster.contains(neighbor)) cluster.add(neighbor);
        }
    }

    private static List<Translation2d> getNeighbors(Translation2d point, List<Translation2d> allPoints) {
        List<Translation2d> neighbors = new ArrayList<>();
        for (Translation2d p : allPoints) {
            if (point.getDistance(p) <= DBSCAN_EPS) {
                neighbors.add(p);
            }
        }
        return neighbors;
    }

    private Command buildFollowCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0) return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        List<Translation2d> balls = new ArrayList<>();
        for (int i = 0; i < rx.length; i++) {
            double dx = rx[i] - robotPos.getX(), dy = ry[i] - robotPos.getY();
            if (dx*dx + dy*dy >= min2) balls.add(new Translation2d(rx[i], ry[i]));
        }
        if (balls.isEmpty()) return null;

        // Run DBSCAN
        List<List<Translation2d>> clusters = dbscan(balls);
        List<Pose2d> poses = new ArrayList<>();
        poses.add(robotPose);

        Translation2d current = robotPos;
        List<List<Translation2d>> remaining = new ArrayList<>(clusters);

        while (!remaining.isEmpty()) {
            final Translation2d ref = current;
            
            // Find nearest cluster based on the closest ball in that cluster
            List<Translation2d> nearestCluster = remaining.stream().min((a, b) -> {
                double distA = a.stream().mapToDouble(ref::getDistance).min().getAsDouble();
                double distB = b.stream().mapToDouble(ref::getDistance).min().getAsDouble();
                return Double.compare(distA, distB);
            }).get();

            remaining.remove(nearestCluster);

            List<Pose2d> clusterPath = clusterToSmartPoints(nearestCluster, ref);
            
            if (!clusterPath.isEmpty()) {
                poses.addAll(clusterPath);
                current = clusterPath.get(clusterPath.size() - 1).getTranslation();
            }
        }

        if (poses.size() < 2) return null;

        Logger.recordOutput("ObjectVision/Waypoints",
            poses.stream().map(p -> new Pose3d(p.getX(), p.getY(), 0, new Rotation3d()))
                .toArray(Pose3d[]::new));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            CONSTRAINTS,
            null,
            new GoalEndState(0.0, poses.get(poses.size() - 1).getRotation())
        );
        path.preventFlipping = true;

        return AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);
    }

    private static List<Pose2d> clusterToSmartPoints(List<Translation2d> cluster, Translation2d from) {
        if (cluster.isEmpty()) return List.of();
        
        // Calculate centroid
        double sumX = 0, sumY = 0;
        for (Translation2d p : cluster) {
            sumX += p.getX();
            sumY += p.getY();
        }
        Translation2d centroid = new Translation2d(sumX / cluster.size(), sumY / cluster.size());

        // Calc ball furthest from robot to drive through the cluster
        Translation2d exit = cluster.stream()
            .max((a, b) -> Double.compare(from.getDistance(a), from.getDistance(b)))
            .get();

        Rotation2d heading = exit.minus(centroid).getAngle();

        List<Pose2d> points = new ArrayList<>();
        points.add(new Pose2d(centroid, heading));
        
        if (centroid.getDistance(exit) > 0.2) {
            points.add(new Pose2d(exit, heading));
        }
        
        return points;
    }

    public Command getPath() {
        return Commands.defer(() -> {
            Command c = buildFollowCommand();
            return c != null ? c : Commands.none();
        }, java.util.Set.of(swerve));
    }

    public Command getDynamicPath() {
        return Commands.defer(() -> {
            Command c = buildFollowCommand();
            return c != null ? c : Commands.none();
        }, java.util.Set.of(swerve))
        .withTimeout(RECALCULATE_PERIOD_S)
        .repeatedly();
    }
}
