package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
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

    private static final double MIN_BALL_DISTANCE_M = 0.3; // Balls closer then this are ignored
    private static final double DBSCAN_EPS = 10; // How close the balls have to be to be in a clump
    private static final int DBSCAN_MIN_PTS = 1; // Min points for a cluster

    private static final double PASS_THROUGH_VEL = 1.5; // How fast the robot should end a sequence. This is good

    private static final double PHYS_MAX_SPEED = 6.8; // Top speed
    private static final double PHYS_MAX_ACCEL = 3; // Max accel
    private static final double PHYS_TURN_DECAY = 2.3; // Speed loss per radian, lower = more turning
    private static final double PHYS_MIN_SPEED = 0.3; // Lowest speed the robot will ever go
    private static final double PHYS_TURN_HARD_STOP = Math.toRadians(100); // 100 degrees, any turns greater then this
                                                                           // get double penalty
    private static final double PHYS_TURN_CLIFF = 8.0; // Pentaly for above param. Adds 8 seconds of time
    private static final double PHYS_ALIGN_BONUS = 1.6; // Bonus for clusters ahead of robot 20 degrees, get (time /
                                                        // 1.6)
    private static final double PHYS_AHEAD_ANGLE = 20; // Degrees fora cluster to be counted as ahead of robot

    private static final double BUDGET_M = 10.0; // Max path lenght in meters. Use auto_time_seconds × average_speed_m/s
    private static final int MAX_BALLS = 300; // Max amount of balls to accept

    private static final double INTAKE_OFFSET_RAD = 0.0;

    // This is for stale vision updates
    private long lastVisionUpdateMs = 0;
    private static final long VISION_MAX_AGE_MS = 500;

    // This is for balls detected on the sides (probaly bad)
    private static final double FIELD_LENGTH_M = 16.46;
    private static final double FIELD_WIDTH_M  = 8.23;
    private static final double FIELD_MARGIN_M = 0.3;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            DriveConstants.maxSpeedMetersPerSec, 3.0,
            Math.PI * 2, Units.degreesToRadians(720));

    private final Pose3d[] ballPosesBuf = new Pose3d[MAX_BALLS];

    private Command kindleWaypointCommand = Commands.none();

    public ObjectVision(Drive drive, ObjectVisionIO io) {
        this.io = io;
        this.swerve = drive;
        for (int i = 0; i < MAX_BALLS; i++)
            ballPosesBuf[i] = new Pose3d();

        io.setWaypointListener(this::updateKindleWaypointsCommand);
    }

    private void updateKindleWaypointsCommand(Pose2d waypoints[]) {
        //System.out.println("TEST 1" + waypoints.length);
        // Safety checsk
        if (waypoints == null || waypoints.length == 0) {
            kindleWaypointCommand = Commands.none();
            return;
        }

        Command sequence = Commands.none();

        for (int i = 0; i < waypoints.length; i++) {
            //System.out.println("TEST 2");
            Pose2d target = waypoints[i];

            sequence = sequence.andThen(
                AutoBuilder.pathfindToPose(
                    target,
                    CONSTRAINTS,
                    PASS_THROUGH_VEL
                )
            );
        }

        System.out.println("SET COMMAND");
        kindleWaypointCommand = sequence;
    }

    private static double trapezoidTime(double dist, double v0, double v1) {
        if (dist < 1e-6)
            return 0.0;
        v0 = Math.min(v0, PHYS_MAX_SPEED);
        v1 = Math.min(v1, PHYS_MAX_SPEED);

        double dUp = (PHYS_MAX_SPEED * PHYS_MAX_SPEED - v0 * v0) / (2 * PHYS_MAX_ACCEL);
        double dDown = (PHYS_MAX_SPEED * PHYS_MAX_SPEED - v1 * v1) / (2 * PHYS_MAX_ACCEL);

        if (dUp + dDown <= dist) {
            double tUp = (PHYS_MAX_SPEED - v0) / PHYS_MAX_ACCEL;
            double tDown = (PHYS_MAX_SPEED - v1) / PHYS_MAX_ACCEL;
            double tFlat = (dist - dUp - dDown) / PHYS_MAX_SPEED;
            return tUp + tFlat + tDown;
        } else {
            double vPeak = Math.sqrt(Math.max(0.0,
                    PHYS_MAX_ACCEL * dist + (v0 * v0 + v1 * v1) / 2.0));
            vPeak = Math.min(vPeak, PHYS_MAX_SPEED);
            double tUp = (vPeak - v0) / PHYS_MAX_ACCEL;
            double tDown = (vPeak - v1) / PHYS_MAX_ACCEL;
            return tUp + tDown;
        }
    }

    private static double turnSpeed(double approachSpeed, double turnRad) {
        return Math.max(PHYS_MIN_SPEED,
                approachSpeed * Math.exp(-PHYS_TURN_DECAY * turnRad));
    }

    private static double[] clusterPhysCost(
            List<Translation2d> cluster, Translation2d fromPos,
            double curSpeed, Double curHdg) {

        Translation2d entry = nearestPoint(cluster, fromPos);
        Translation2d exit_ = farthestPoint(cluster, fromPos);

        double dApproach = fromPos.getDistance(entry);
        double dThrough = entry.getDistance(exit_);

        double turn = (curHdg != null)
                ? headingDiff(curHdg, angleTo(fromPos, entry))
                : 0.0;

        // Layer 1: exponential speed loss
        double vEntry = turnSpeed(curSpeed > 0 ? curSpeed : PHYS_MAX_SPEED, turn);
        double vExit  = vEntry;

        double tApproach = trapezoidTime(dApproach, curSpeed, vEntry);
        double tThrough = trapezoidTime(dThrough, vEntry, vExit);
        double totalTime = tApproach + tThrough;

        if (turn > PHYS_TURN_HARD_STOP) {
            double excess = turn - PHYS_TURN_HARD_STOP;
            totalTime += PHYS_TURN_CLIFF * (1.0 - Math.exp(-excess * 3.0));
        }

        if (curHdg != null && turn < Math.toRadians(PHYS_AHEAD_ANGLE)) {
            totalTime /= PHYS_ALIGN_BONUS;
        }

        return new double[] { totalTime, vExit };
    }

    private Command buildVelocityOpPathCommand(double budgetMeters, double passThroughVel, int maxClusters) {
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty()) return null;

        Translation2d robotPos = swerve.getPose().getTranslation();
        List<List<Translation2d>> clusters = dbscan(balls);
        int n = clusters.size();

        List<Integer>  seq       = new ArrayList<>();
        boolean[]      visited   = new boolean[n];
        double         spentDist = 0.0;
        Translation2d  cur       = robotPos;
        double         curSpeed  = 0.0;    // robot starts stationary
        Double         curHdg    = null;   // no heading until first move

        boolean anyAdded = true;
        while (anyAdded) {
            anyAdded = false;

            if (maxClusters > 0 && seq.size() >= maxClusters) break;

            int    bestIdx   = -1;
            double bestScore = -1.0;
            // double bestTCost = 0.0;
            double bestSpd   = 0.0;

            for (int i = 0; i < n; i++) {
                if (visited[i]) continue;
                List<Translation2d> c = clusters.get(i);

                double distCost = traverseDist(c, cur);
                if (spentDist + distCost > budgetMeters) continue;

                // Hard-reject near-reversals as long as other options exist
                if (curHdg != null) {
                    
                    Translation2d entry = nearestPoint(c, cur);
                    double turn = headingDiff(curHdg, angleTo(cur, entry));
                    final int currentI = i;
                    long remaining = java.util.stream.IntStream.range(0, n)
                            .filter(j -> !visited[j] && j != currentI).count();
                    if (turn > Math.toRadians(150) && remaining > 0) continue;
                }

                double[] phys = clusterPhysCost(c, cur, curSpeed, curHdg);
                double tCost = phys[0];
                double exitSpd = phys[1];
                double score = c.size() / Math.max(tCost, 0.01);

                if (score > bestScore) {
                    bestScore = score;
                    bestIdx   = i;
                    bestSpd   = exitSpd;
                }
                
            }

            if (bestIdx < 0) break;

            Translation2d ex = exitPos(clusters.get(bestIdx), cur);
            curHdg    = angleTo(cur, ex);
            curSpeed  = bestSpd;
            spentDist += traverseDist(clusters.get(bestIdx), cur);
            cur = ex;
            seq.add(bestIdx);
            visited[bestIdx] = true;
            anyAdded = true;
        }

        if (seq.isEmpty()) return null;

        Translation2d[] pos = new Translation2d[seq.size() + 1];
        pos[0] = robotPos;
        for (int k = 0; k < seq.size(); k++)
            pos[k + 1] = exitPos(clusters.get(seq.get(k)), pos[k]);

        boolean improved = true;
        while (improved) {
            improved = false;
            outer:
            for (int i = 0; i < seq.size() - 1; i++) {
                for (int j = i + 1; j < seq.size(); j++) {
                    double dOrigA = pos[i].getDistance(
                        nearestPoint(clusters.get(seq.get(i)), pos[i]));
                    double dOrigB = (j + 1 < seq.size())
                        ? pos[j].getDistance(
                            nearestPoint(clusters.get(seq.get(j + 1)), pos[j]))
                        : 0.0;

                    // Compute what pos[i+1] would be after the swap
                    Translation2d swappedExit = exitPos(clusters.get(seq.get(j)), pos[i]);
                    double dSwapA = pos[i].getDistance(
                        nearestPoint(clusters.get(seq.get(j)), pos[i]));
                    double dSwapB = (j + 1 < seq.size())
                        ? swappedExit.getDistance(
                            nearestPoint(clusters.get(seq.get(j + 1)), swappedExit))
                        : 0.0;

                    if (dSwapA + dSwapB < dOrigA + dOrigB - 0.05) {
                        // Reverse the segment [i..j] in seq
                        List<Integer> sub = new ArrayList<>(seq.subList(i, j + 1));
                        java.util.Collections.reverse(sub);
                        for (int k = 0; k < sub.size(); k++) seq.set(i + k, sub.get(k));

                        // Recompute positions for the full affected range
                        for (int k = i; k < seq.size(); k++)
                            pos[k + 1] = exitPos(clusters.get(seq.get(k)), pos[k]);

                        improved = true;
                        break outer;
                    }
                }
            }
        }

        List<Pose2d> targets = new ArrayList<>();
        cur = robotPos;
        for (int idx : seq) {
            List<Pose2d> pts = clusterToSpinePoints(clusters.get(idx), cur);
            targets.addAll(pts);
            if (!pts.isEmpty())
                cur = pts.get(pts.size() - 1).getTranslation();
        }

        if (targets.isEmpty()) return null;

        Logger.recordOutput("ObjectVision/VelocityOpWaypoints",
            targets.stream()
                .map(p -> new Pose3d(p.getX(), p.getY(), 0.0, new Rotation3d()))
                .toArray(Pose3d[]::new));

        Command sequence = Commands.none();
        for (int i = 0; i < targets.size(); i++) {
            double vel = (i == targets.size() - 1) ? 0.0 : passThroughVel;
            sequence = sequence.andThen(
                AutoBuilder.pathfindToPose(targets.get(i), CONSTRAINTS, vel));
        }
        return sequence;
    }

    public Command driveVelocityOpPath(double budgetMeters, double passThroughVel, int maxClusters) {
        return Commands.deferredProxy(() -> {
            Command c = buildVelocityOpPathCommand(budgetMeters, passThroughVel, maxClusters);
            return c != null ? c : Commands.none();
        });
    }

    public Command driveVelocityOpPath() {
        return driveVelocityOpPath(BUDGET_M, PASS_THROUGH_VEL, -1);
    }

    private Command buildDriveToClosestBallCommand() {
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        Translation2d robotPos = swerve.getPose().getTranslation();

        Translation2d closest = balls.stream()
                .min((a, b) -> Double.compare(robotPos.getDistance(a), robotPos.getDistance(b)))
                .orElse(null);
        if (closest == null)
            return null;

        Rotation2d heading = closest.minus(robotPos).getAngle().plus(new Rotation2d(INTAKE_OFFSET_RAD));
        Pose2d target = new Pose2d(closest, heading);

        Logger.recordOutput("ObjectVision/ClosestBallTarget",
                new Pose3d(closest.getX(), closest.getY(), 0.0, new Rotation3d()));

        return AutoBuilder.pathfindToPose(target, CONSTRAINTS, 0.0);
    }

    public Command driveToClosestBall() {
        return Commands.deferredProxy(() -> {
            Command c = buildDriveToClosestBallCommand();
            return c != null ? c : Commands.none();
        });
    }

    private static double angleTo(Translation2d from, Translation2d to) {
        return Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
    }

    private static double headingDiff(double h1, double h2) {
        double diff = ((h2 - h1) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        return Math.min(diff, 2 * Math.PI - diff);
    }

    private static double traverseDist(List<Translation2d> cluster, Translation2d from) {
        Translation2d entry = nearestPoint(cluster, from);
        Translation2d exit = farthestPoint(cluster, from);
        return from.getDistance(entry) + entry.getDistance(exit);
    }

    private static Translation2d exitPos(List<Translation2d> cluster, Translation2d from) {
        if (cluster.size() == 1)
            return cluster.get(0);
        Translation2d sA = nearestPoint(cluster, from);
        Translation2d sB = farthestPoint(cluster, from);
        return (from.getDistance(sB) >= from.getDistance(sA)) ? sB : sA;
    }

    private static Translation2d nearestPoint(List<Translation2d> cluster, Translation2d ref) {
        return cluster.stream()
                .min((a, b) -> Double.compare(ref.getDistance(a), ref.getDistance(b)))
                .orElseThrow();
    }

    private static Translation2d farthestPoint(List<Translation2d> cluster, Translation2d ref) {
        return cluster.stream()
                .max((a, b) -> Double.compare(ref.getDistance(a), ref.getDistance(b)))
                .orElseThrow();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectVision", inputs);

        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0) {
            Logger.recordOutput("ObjectVision/Balls", new Pose3d[0]);
            return;
        }
        lastVisionUpdateMs = System.currentTimeMillis();

        int n = Math.min(rx.length, MAX_BALLS);
        for (int i = 0; i < n; i++)
            ballPosesBuf[i] = new Pose3d(rx[i], ry[i], 0.102, new Rotation3d());
        Logger.recordOutput("ObjectVision/Balls", java.util.Arrays.copyOf(ballPosesBuf, n));
    }

    private static List<Pose2d> clusterToSpinePoints(
            List<Translation2d> cluster, Translation2d from) {

        if (cluster.isEmpty())
            return List.of();

        if (cluster.size() == 1) {
            Translation2d p = cluster.get(0);
            return List.of(new Pose2d(p, p.minus(from).getAngle().plus(new Rotation2d(INTAKE_OFFSET_RAD))));
        }

        double cx = 0.0, cy = 0.0;
        for (Translation2d p : cluster) {
            cx += p.getX();
            cy += p.getY();
        }
        cx /= cluster.size();
        cy /= cluster.size();

        double cxx = 0.0, cxy = 0.0, cyy = 0.0;
        for (Translation2d p : cluster) {
            double dx = p.getX() - cx, dy = p.getY() - cy;
            cxx += dx * dx;
            cxy += dx * dy;
            cyy += dy * dy;
        }

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
        if (axLen < 1e-9) {
            axX = 1.0;
            axY = 0.0;
        } else {
            axX /= axLen;
            axY /= axLen;
        }

        double minT = Double.MAX_VALUE, maxT = -Double.MAX_VALUE;
        for (Translation2d p : cluster) {
            double t = (p.getX() - cx) * axX + (p.getY() - cy) * axY;
            if (t < minT)
                minT = t;
            if (t > maxT)
                maxT = t;
        }

        Translation2d spineA = new Translation2d(cx + minT * axX, cy + minT * axY);
        Translation2d spineB = new Translation2d(cx + maxT * axX, cy + maxT * axY);

        if (from.getDistance(spineB) < from.getDistance(spineA)) {
            Translation2d tmp = spineA;
            spineA = spineB;
            spineB = tmp;
            axX = -axX;
            axY = -axY;
        }

        return List.of(new Pose2d(spineB, new Rotation2d(axX, axY).plus(new Rotation2d(INTAKE_OFFSET_RAD))));
    }

    private Command buildDriveThroughClumpCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0)
            return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();
        // double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        List<List<Translation2d>> clusters = dbscan(balls);

        // Find only the nearest cluster (don't visit all of them)
        List<Translation2d> nearest = clusters.stream().min((a, b) -> {
            double dA = a.stream().mapToDouble(robotPos::getDistance).min().getAsDouble();
            double dB = b.stream().mapToDouble(robotPos::getDistance).min().getAsDouble();
            return Double.compare(dA, dB);
        }).orElse(null);
        if (nearest == null)
            return null;

        // Get centroid and exit point so the robot drives "through" the cluster
        List<Pose2d> targets = clusterToSmartPoints(nearest, robotPos);
        if (targets.isEmpty())
            return null;

        Command sequence = Commands.none();
        for (Pose2d target : targets)
            sequence = sequence.andThen(AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL));
        return sequence;
    }

    private Command buildDriveToClumpCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0)
            return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();
        // double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        // Collect valid balls (not too close to the robot)
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        // Cluster and order greedily from the robot outward
        List<List<Translation2d>> clusters = dbscan(balls);
        List<Pose2d> targets = orderedClusterTargets(clusters, robotPos);
        if (targets.isEmpty()) {
            return Commands.none();
        }
        return AutoBuilder.pathfindToPose(targets.get(0), CONSTRAINTS, 0.0);
    }

    private List<Translation2d> getValidBalls() {
        if (System.currentTimeMillis() - lastVisionUpdateMs > VISION_MAX_AGE_MS) {
            return List.of(); // data is stale, pretend we see nothing
        }
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null)
            return List.of();

        Translation2d robotPos = swerve.getPose().getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        List<Translation2d> balls = new ArrayList<>();
        for (int i = 0; i < rx.length; i++) {
            double dx = rx[i] - robotPos.getX(), dy = ry[i] - robotPos.getY();
            double bx = rx[i], by = ry[i];
            if (bx < FIELD_MARGIN_M || bx > FIELD_LENGTH_M - FIELD_MARGIN_M
                || by < FIELD_MARGIN_M || by > FIELD_WIDTH_M - FIELD_MARGIN_M) {
                continue; // vision noise/out of bounds
            }

            if (dx * dx + dy * dy >= min2)
                balls.add(new Translation2d(rx[i], ry[i]));
        }
        return balls;
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

    private static List<Pose2d> clusterToSmartPoints(
            List<Translation2d> cluster, Translation2d from) {

        if (cluster.isEmpty())
            return List.of();

        Translation2d exit = cluster.stream()
                .max((a, b) -> Double.compare(from.getDistance(a), from.getDistance(b)))
                .get();

        Rotation2d heading = exit.minus(from).getAngle().plus(new Rotation2d(INTAKE_OFFSET_RAD));
        return List.of(new Pose2d(exit, heading));
    }

    private static List<List<Translation2d>> dbscan(List<Translation2d> points) {
        List<List<Translation2d>> clusters = new ArrayList<>();
        Set<Translation2d> visited = new HashSet<>();
        Set<Translation2d> noise = new HashSet<>();

        for (Translation2d point : points) {
            if (visited.contains(point))
                continue;
            visited.add(point);

            List<Translation2d> neighbors = getNeighbors(point, points);
            if (neighbors.size() < DBSCAN_MIN_PTS) {
                noise.add(point);
            } else {
                LinkedHashSet<Translation2d> cluster = new LinkedHashSet<>();
                expandCluster(point, neighbors, cluster, visited, points);
                clusters.add(new ArrayList<>(cluster));
            }
        }

        // Treat noise points as their own single-ball clusters so they aren't dropped
        for (Translation2d n : noise)
            clusters.add(List.of(n));

        return clusters;
    }

    private static void expandCluster(
            Translation2d point, List<Translation2d> neighbors,
            LinkedHashSet<Translation2d> cluster,   // ← was List
            Set<Translation2d> visited,
            List<Translation2d> allPoints) {

        cluster.add(point);
        for (int i = 0; i < neighbors.size(); i++) {
            Translation2d neighbor = neighbors.get(i);
            if (!visited.contains(neighbor)) {
                visited.add(neighbor);
                List<Translation2d> next = getNeighbors(neighbor, allPoints);
                if (next.size() >= DBSCAN_MIN_PTS)
                    neighbors.addAll(next);
            }
            cluster.add(neighbor);
        }
    }

    private static List<Translation2d> getNeighbors(
            Translation2d point, List<Translation2d> allPoints) {

        List<Translation2d> neighbors = new ArrayList<>();
        for (Translation2d p : allPoints)
            if (point.getDistance(p) <= DBSCAN_EPS)
                neighbors.add(p);
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

    public Command kindleCommand() {

        return Commands.parallel(
                    Commands.runOnce(() -> { 
                        Commands.print("RETURNED COMMAND");
                    }),
                    Commands.deferredProxy(() -> kindleWaypointCommand));
    }
}