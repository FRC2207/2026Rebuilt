package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;

public class ObjectVision extends SubsystemBase {
    private final Drive swerve;
    private final ObjectVisionIO io;
    private final ObjectVisionIOInputsAutoLogged inputs = new ObjectVisionIOInputsAutoLogged();

    private static final double MIN_BALL_DISTANCE_M = 0.3;
    private static final double DBSCAN_EPS = 0.5;
    private static final int DBSCAN_MIN_PTS = 4;
    private static final double PASS_THROUGH_VEL = 1.5;
    private static final double MIN_SCORE_BALLS_PER_METER = 1.5;
    private static final int MAX_CLUSTERS_TO_VISIT = 4;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            DriveConstants.maxSpeedMetersPerSec, 3.0,
            Math.PI * 2, Units.degreesToRadians(720));

    private static final int MAX_BALLS = 300;
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
        if (rx == null || ry == null) {
            Logger.recordOutput("ObjectVision/Balls", new Pose3d[0]);
            return;
        }
        int n = Math.min(rx.length, MAX_BALLS);
        for (int i = 0; i < n; i++)
            ballPosesBuf[i] = new Pose3d(rx[i], ry[i], 0.0, new Rotation3d());
        Logger.recordOutput("ObjectVision/Balls", java.util.Arrays.copyOf(ballPosesBuf, n));
    }

    private static List<Pose2d> clusterToSpinePoints(
            List<Translation2d> cluster, Translation2d from) {

        if (cluster.isEmpty()) return List.of();

        if (cluster.size() == 1) {
            Translation2d p = cluster.get(0);
            return List.of(new Pose2d(p, p.minus(from).getAngle()));
        }

        // ── 1. Centroid ───────────────────────────────────────────────────────────
        double cx = 0.0, cy = 0.0;
        for (Translation2d p : cluster) { cx += p.getX(); cy += p.getY(); }
        cx /= cluster.size();
        cy /= cluster.size();

        // ── 2. 2×2 covariance ────────────────────────────────────────────────────
        double cxx = 0.0, cxy = 0.0, cyy = 0.0;
        for (Translation2d p : cluster) {
            double dx = p.getX() - cx, dy = p.getY() - cy;
            cxx += dx * dx;
            cxy += dx * dy;
            cyy += dy * dy;
        }

        // ── 3. Principal eigenvector (analytical) ────────────────────────────────
        double half = (cxx + cyy) * 0.5;
        double disc = Math.sqrt(Math.max(0.0, half * half - (cxx * cyy - cxy * cxy)));
        double lam1 = half + disc;

        double axX, axY;
        if (Math.abs(cxy) > 1e-9) {
            axX = lam1 - cyy;
            axY = cxy;
        } else {
            axX = (cxx >= cyy) ? 1.0 : 0.0;
            axY = (cxx >= cyy) ? 0.0 : 1.0;
        }
        double axLen = Math.hypot(axX, axY);
        if (axLen < 1e-9) { axX = 1.0; axY = 0.0; }
        else              { axX /= axLen; axY /= axLen; }

        // ── 4. Find the far end of the spine from the robot ──────────────────────
        double minT = Double.MAX_VALUE, maxT = -Double.MAX_VALUE;
        for (Translation2d p : cluster) {
            double t = (p.getX() - cx) * axX + (p.getY() - cy) * axY;
            if (t < minT) minT = t;
            if (t > maxT) maxT = t;
        }

        Translation2d spineA = new Translation2d(cx + minT * axX, cy + minT * axY);
        Translation2d spineB = new Translation2d(cx + maxT * axX, cy + maxT * axY);

        // Ensure spineB is the far end from the robot (exit side)
        if (from.getDistance(spineB) < from.getDistance(spineA)) {
            Translation2d tmp = spineA; spineA = spineB; spineB = tmp;
            // Flip axis direction so heading still points toward exit
            axX = -axX; axY = -axY;
        }

        // ── 5. Single waypoint: far end, heading along the spine ─────────────────
        // Using the spine axis (not robot→exit) means the robot exits already
        // aligned for the next cluster — no heading discontinuities mid-chain.
        return List.of(new Pose2d(spineB, new Rotation2d(axX, axY)));
    }

    private Command buildDriveThroughClumpCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0) return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty()) return null;

        List<List<Translation2d>> clusters = dbscan(balls);

        // Find only the nearest cluster (don't visit all of them)
        List<Translation2d> nearest = clusters.stream().min((a, b) -> {
            double dA = a.stream().mapToDouble(robotPos::getDistance).min().getAsDouble();
            double dB = b.stream().mapToDouble(robotPos::getDistance).min().getAsDouble();
            return Double.compare(dA, dB);
        }).orElse(null);
        if (nearest == null) return null;

        // Get centroid and exit point so the robot drives "through" the cluster
        List<Pose2d> targets = clusterToSmartPoints(nearest, robotPos);
        if (targets.isEmpty()) return null;

        Command sequence = Commands.none();
        for (Pose2d target : targets)
            sequence = sequence.andThen(AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL));
        return sequence;
    }

    private Command buildDriveToClumpCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0) return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos  = robotPose.getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;
        
        // Collect valid balls (not too close to the robot)
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty()) return null;
        

        // Cluster and order greedily from the robot outward
        List<List<Translation2d>> clusters = dbscan(balls);
        List<Pose2d> targets = orderedClusterTargets(clusters, robotPos);

        return AutoBuilder.pathfindToPose(targets.get(0), CONSTRAINTS, 0.0);
    }

    private List<Translation2d> getValidBalls() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null) return List.of();

        Translation2d robotPos = swerve.getPose().getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        List<Translation2d> balls = new ArrayList<>();
        for (int i = 0; i < rx.length; i++) {
            double dx = rx[i] - robotPos.getX(), dy = ry[i] - robotPos.getY();
            if (dx * dx + dy * dy >= min2)
                balls.add(new Translation2d(rx[i], ry[i]));
        }
        return balls;
    }

    private Command buildFollowCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0) return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos  = robotPose.getTranslation();
        double min2      = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        // Collect valid balls (not too close to the robot)
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty()) return null;

        // Cluster and order greedily from the robot outward
        List<List<Translation2d>> clusters = dbscan(balls);
        List<Pose2d> targets = orderedClusterTargets(clusters, robotPos);
        if (targets.isEmpty()) return null;

        Logger.recordOutput("ObjectVision/Waypoints",
                targets.stream()
                       .map(p -> new Pose3d(p.getX(), p.getY(), 0, new Rotation3d()))
                       .toArray(Pose3d[]::new));

        Command sequence = Commands.none();
        for (Pose2d target : targets) {
            sequence = sequence.andThen(
                AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL)
            );
        }
        return sequence;
    }

    private static List<Pose2d> orderedClusterTargets(
            List<List<Translation2d>> clusters, Translation2d from) {

        List<Pose2d> result = new ArrayList<>();
        List<List<Translation2d>> remaining = new ArrayList<>(clusters);
        Translation2d current = from;

        while (!remaining.isEmpty()) {
            final Translation2d ref = current;

            // Nearest cluster = cluster whose closest ball is nearest to ref
            List<Translation2d> nearest = remaining.stream().min((a, b) -> {
                double dA = a.stream().mapToDouble(ref::getDistance).min().getAsDouble();
                double dB = b.stream().mapToDouble(ref::getDistance).min().getAsDouble();
                return Double.compare(dA, dB);
            }).get();

            remaining.remove(nearest);

            List<Pose2d> clusterPoses = clusterToSmartPoints(nearest, ref);
            result.addAll(clusterPoses);

            if (!clusterPoses.isEmpty())
                current = clusterPoses.get(clusterPoses.size() - 1).getTranslation();
        }
        return result;
    }

    // This uses centroid, and exit point.
    // private static List<Pose2d> clusterToSmartPoints(
    //         List<Translation2d> cluster, Translation2d from) {

    //     if (cluster.isEmpty()) return List.of();

    //     double sumX = 0, sumY = 0;
    //     for (Translation2d p : cluster) { sumX += p.getX(); sumY += p.getY(); }
    //     Translation2d centroid = new Translation2d(sumX / cluster.size(), sumY / cluster.size());

    //     Translation2d exit = cluster.stream()
    //             .max((a, b) -> Double.compare(from.getDistance(a), from.getDistance(b)))
    //             .get();

    //     Rotation2d heading = exit.minus(centroid).getAngle();

    //     List<Pose2d> points = new ArrayList<>();
    //     points.add(new Pose2d(centroid, heading));
    //     if (centroid.getDistance(exit) > 0.2)
    //         points.add(new Pose2d(exit, heading));

    //     return points;
    // }

 private Command buildSpinePathCommand() {
    List<Translation2d> balls = getValidBalls();
    if (balls.isEmpty()) return null;

    Translation2d robotPos = swerve.getPose().getTranslation();
    List<List<Translation2d>> clusters = dbscan(balls);

    // ── 1. Score every cluster and drop the low-value ones ───────────────────
    //   score = cluster size / distance to nearest ball in the cluster
    //   This rewards large nearby clusters and penalises small or distant ones.
    List<List<Translation2d>> candidates = new ArrayList<>();
    for (List<Translation2d> cluster : clusters) {
        double nearestDist = cluster.stream()
                .mapToDouble(robotPos::getDistance)
                .min().getAsDouble();

        // Avoid divide-by-zero if robot is sitting on a ball
        double dist  = Math.max(nearestDist, 0.1);
        double score = cluster.size() / dist;

        if (score >= MIN_SCORE_BALLS_PER_METER)
            candidates.add(cluster);
    }

    // Always keep at least the single best cluster so the command does something
    if (candidates.isEmpty()) {
        clusters.stream().max((a, b) -> {
            double dA = a.stream().mapToDouble(robotPos::getDistance).min().getAsDouble();
            double dB = b.stream().mapToDouble(robotPos::getDistance).min().getAsDouble();
            double sA = a.size() / Math.max(dA, 0.1);
            double sB = b.size() / Math.max(dB, 0.1);
            return Double.compare(sA, sB);
        }).ifPresent(candidates::add);
    }

    // ── 2. Greedy nearest-first chain, but only among candidates ─────────────
    List<Pose2d> targets   = new ArrayList<>();
    List<List<Translation2d>> remaining = new ArrayList<>(candidates);
    Translation2d current  = robotPos;
    int visited            = 0;

    while (!remaining.isEmpty() && visited < MAX_CLUSTERS_TO_VISIT) {
        final Translation2d ref = current;

        List<Translation2d> nearest = remaining.stream().min((a, b) -> {
            double dA = a.stream().mapToDouble(ref::getDistance).min().getAsDouble();
            double dB = b.stream().mapToDouble(ref::getDistance).min().getAsDouble();
            return Double.compare(dA, dB);
        }).get();

        remaining.remove(nearest);

        List<Pose2d> pts = clusterToSpinePoints(nearest, ref);
        targets.addAll(pts);
        if (!pts.isEmpty())
            current = pts.get(pts.size() - 1).getTranslation();

        visited++;
    }

    if (targets.isEmpty()) return null;

    Logger.recordOutput("ObjectVision/SpineWaypoints",
        targets.stream()
               .map(p -> new Pose3d(p.getX(), p.getY(), 0.0, new Rotation3d()))
               .toArray(Pose3d[]::new));

    Command sequence = Commands.none();
    for (Pose2d target : targets)
        sequence = sequence.andThen(
            AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL));
    return sequence;
}
    public Command driveSpinePath() {
        return Commands.deferredProxy(() -> {
            Command c = buildSpinePathCommand();
            return c != null ? c : Commands.none();
        });
    }

    private static List<Pose2d> clusterToSmartPoints(
            List<Translation2d> cluster, Translation2d from) {

        if (cluster.isEmpty()) return List.of();

        Translation2d exit = cluster.stream()
                .max((a, b) -> Double.compare(from.getDistance(a), from.getDistance(b)))
                .get();

        Rotation2d heading = exit.minus(from).getAngle();

        return List.of(new Pose2d(exit, heading));
    }

    private static List<List<Translation2d>> dbscan(List<Translation2d> points) {
        List<List<Translation2d>> clusters = new ArrayList<>();
        Set<Translation2d>        visited  = new HashSet<>();
        Set<Translation2d>        noise    = new HashSet<>();

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

        // Treat noise points as their own single-ball clusters so they aren't dropped
        for (Translation2d n : noise)
            clusters.add(List.of(n));

        return clusters;
    }

    private static void expandCluster(
            Translation2d point, List<Translation2d> neighbors,
            List<Translation2d> cluster, Set<Translation2d> visited,
            List<Translation2d> allPoints) {

        cluster.add(point);
        for (int i = 0; i < neighbors.size(); i++) {
            Translation2d neighbor = neighbors.get(i);
            if (!visited.contains(neighbor)) {
                visited.add(neighbor);
                List<Translation2d> next = getNeighbors(neighbor, allPoints);
                if (next.size() >= DBSCAN_MIN_PTS) neighbors.addAll(next);
            }
            if (!cluster.contains(neighbor)) cluster.add(neighbor);
        }
    }

    private static List<Translation2d> getNeighbors(
            Translation2d point, List<Translation2d> allPoints) {

        List<Translation2d> neighbors = new ArrayList<>();
        for (Translation2d p : allPoints)
            if (point.getDistance(p) <= DBSCAN_EPS) neighbors.add(p);
        return neighbors;
    }

    public Command driveToClump() {
        return Commands.deferredProxy(() -> {
            Command c = buildDriveToClumpCommand();
            return c != null ? c : Commands.none();
        });
    }

    public Command driveThroughClump() {
        return Commands.deferredProxy(() -> {
            Command c = buildDriveThroughClumpCommand();
            return c != null ? c : Commands.none();
        });
    }

    public Command getPath() {
        return Commands.deferredProxy(() -> {
            Command c = buildFollowCommand();
            return c != null ? c : Commands.none();
        });
    }

    public Command getDynamicPath() {
        return Commands.deferredProxy(() -> {
            Command c = buildDriveThroughClumpCommand();
            return c != null ? c : Commands.none();
        }).withTimeout(0.1).repeatedly();
    }
}