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

    private static final double MIN_BALL_DISTANCE_M = 0.3; // Balls closer then this are ignored
    private static final double DBSCAN_EPS = 0.5; // How close the balls have to be to be in a clump
    private static final int DBSCAN_MIN_PTS = 4; // Min points for a cluster

    private static final double PASS_THROUGH_VEL = 1.5; // How fast the robot should end a sequence. This is good

    // Momentum params
    private static final double MOMENTUM_MAX_TURN_RAD = Math.toRadians(100); // Reject clumps that 110 degrees
    private static final double MOMENTUM_FILL_MAX_TURN = Math.toRadians(120); // Second pass reject clumps hat are 140
                                                                              // degrees
    private static final double MOMENTUM_STRAGGLER_DIST = 1; // At end of path, control how far robot will go for new
                                                             // clumps
    private static final double MOMENTUM_HEAD_PEN = 10; // Each radian of "turn" add that many "virutal meteress"
    private static final double MOMENTUM_EMA_ALPHA = 0.55; // How quickly the smoothed heading cahnges the actual
                                                           // direction of travel
    private static final double MOMENTUM_RELAX_STEP = Math.toRadians(30); // If fails, reduces angles by 30 for awkward
                                                                          // feild setups
    private static final double PHYS_MAX_SPEED = 6.8;
    private static final double PHYS_MAX_ACCEL = 3;
    private static final double PHYS_TURN_DECAY = 2;
    private static final double PHYS_MIN_SPEED = 0.3;
    private static final double PHYS_TURN_HARD_STOP = 2.1; // ~120 degrees
    private static final double PHYS_TURN_CLIFF = 8.0; // seconds penalty
    private static final double PHYS_ALIGN_BONUS = 1.6; // time divisor for <20deg turns

    private static final double BUDGET_M = 16.0; // Max path lenght in meters. Use auto_time_seconds × average_speed_m/s
    private static final int MAX_BALLS = 300; // Max amount of balls to accept

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            DriveConstants.maxSpeedMetersPerSec, 3.0,
            Math.PI * 2, Units.degreesToRadians(720));

    private final Pose3d[] ballPosesBuf = new Pose3d[MAX_BALLS];

    public ObjectVision(Drive drive, ObjectVisionIO io) {
        this.io = io;
        this.swerve = drive;
        for (int i = 0; i < MAX_BALLS; i++)
            ballPosesBuf[i] = new Pose3d();
    }

    private static double trapezoidTime(double dist, double v0, double v1) {
        if (dist < 1e-6) return 0.0;
        v0 = Math.min(v0, PHYS_MAX_SPEED);
        v1 = Math.min(v1, PHYS_MAX_SPEED);

        double dUp   = (PHYS_MAX_SPEED * PHYS_MAX_SPEED - v0 * v0) / (2 * PHYS_MAX_ACCEL);
        double dDown = (PHYS_MAX_SPEED * PHYS_MAX_SPEED - v1 * v1) / (2 * PHYS_MAX_ACCEL);

        if (dUp + dDown <= dist) {
            // Trapezoidal — robot reaches full speed
            double tUp   = (PHYS_MAX_SPEED - v0) / PHYS_MAX_ACCEL;
            double tDown = (PHYS_MAX_SPEED - v1) / PHYS_MAX_ACCEL;
            double tFlat = (dist - dUp - dDown) / PHYS_MAX_SPEED;
            return tUp + tFlat + tDown;
        } else {
            // Triangular — too short to reach full speed
            double vPeak = Math.sqrt(Math.max(0.0,
                    PHYS_MAX_ACCEL * dist + (v0 * v0 + v1 * v1) / 2.0));
            vPeak      = Math.min(vPeak, PHYS_MAX_SPEED);
            double tUp   = (vPeak - v0) / PHYS_MAX_ACCEL;
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
        double dThrough  = entry.getDistance(exit_);

        double turn = (curHdg != null)
                ? headingDiff(curHdg, angleTo(fromPos, entry))
                : 0.0;

        // Layer 1: exponential speed loss
        double vEntry = turnSpeed(PHYS_MAX_SPEED, turn);
        double vExit  = vEntry;

        double tApproach = trapezoidTime(dApproach, curSpeed, vEntry);
        double tThrough  = trapezoidTime(dThrough,  vEntry,  vExit);
        double totalTime = tApproach + tThrough;

        // Layer 2: hard cliff past ~120 degrees
        if (turn > PHYS_TURN_HARD_STOP) {
            totalTime += PHYS_TURN_CLIFF;
        }

        // Layer 3: alignment bonus for nearly-straight runs (<20 degrees)
        if (curHdg != null && turn < Math.toRadians(20)) {
            totalTime /= PHYS_ALIGN_BONUS;
        }

        return new double[]{ totalTime, vExit };
    }

    private Command buildFastOpPathCommand() {
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        Translation2d robotPos = swerve.getPose().getTranslation();
        List<List<Translation2d>> clusters = dbscan(balls);
        int n = clusters.size();

        List<Integer> seq = new ArrayList<>();
        boolean[] visited = new boolean[n];
        double spent = 0.0;
        Translation2d cur = robotPos;

        boolean anyAdded = true;
        while (anyAdded) {
            anyAdded = false;
            int bestIdx = -1;
            double bestRatio = -1.0;
            double bestCost = 0.0;

            for (int i = 0; i < n; i++) {
                if (visited[i])
                    continue;
                List<Translation2d> c = clusters.get(i);

                double cost = traverseDist(c, cur);
                if (spent + cost > BUDGET_M)
                    continue;

                double ratio = (c.size() * c.size()) / Math.max(cost, 0.01);
                if (ratio > bestRatio) {
                    bestRatio = ratio;
                    bestIdx = i;
                    bestCost = cost;
                }
            }

            if (bestIdx >= 0) {
                seq.add(bestIdx);
                visited[bestIdx] = true;
                spent += bestCost;
                cur = exitPos(clusters.get(bestIdx), cur);
                anyAdded = true;
            }
        }

        if (seq.isEmpty())
            return null;

        boolean improved = true;
        while (improved) {
            improved = false;
            outer: for (int i = 0; i < seq.size() - 1; i++) {
                for (int j = i + 2; j < seq.size(); j++) {

                    Translation2d posI = getSeqPos(seq, i, clusters, robotPos);
                    Translation2d posJ = getSeqPos(seq, j - 1, clusters, robotPos);

                    Translation2d nearI = nearestPoint(clusters.get(seq.get(i)), posI);
                    Translation2d nearJ = nearestPoint(clusters.get(seq.get(j)), posJ);
                    Translation2d exitIp1 = exitPos(clusters.get(seq.get(i)), posI);

                    double dOrig = posI.getDistance(nearI)
                            + exitIp1.getDistance(nearJ);

                    Translation2d nearJfromI = nearestPoint(clusters.get(seq.get(j)), posI);
                    Translation2d exitJfromI = exitPos(clusters.get(seq.get(j)), posI);
                    Translation2d nearIfromJ = nearestPoint(clusters.get(seq.get(i)), posI);

                    double dRev = posI.getDistance(nearJfromI)
                            + exitJfromI.getDistance(nearIfromJ);

                    if (dRev < dOrig - 0.05) {
                        List<Integer> sub = new ArrayList<>(seq.subList(i, j + 1));
                        java.util.Collections.reverse(sub);
                        for (int k = 0; k < sub.size(); k++)
                            seq.set(i + k, sub.get(k));
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

        if (targets.isEmpty())
            return null;

        Logger.recordOutput("ObjectVision/FastOpWaypoints",
                targets.stream()
                        .map(p -> new Pose3d(p.getX(), p.getY(), 0.0, new Rotation3d()))
                        .toArray(Pose3d[]::new));

        Command sequence = Commands.none();
        for (Pose2d target : targets)
            sequence = sequence.andThen(
                    AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL));
        return sequence;
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
            double bestTCost = 0.0;
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

                double[] phys  = clusterPhysCost(c, cur, curSpeed, curHdg);
                double tCost   = phys[0];
                double exitSpd = phys[1];
                double score   = (c.size() * c.size()) / Math.max(tCost, 0.01);

                if (score > bestScore) {
                    bestScore = score;
                    bestIdx   = i;
                    bestTCost = tCost;
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

        // ── Single 2-opt pass (geometry-based) ───────────────────────────────────
        boolean improved = true;
        while (improved) {
            improved = false;
            outer:
            for (int i = 0; i < seq.size() - 1; i++) {
                for (int j = i + 2; j < seq.size(); j++) {
                    Translation2d posI    = getSeqPos(seq, i,     clusters, robotPos);
                    Translation2d exitIp1 = exitPos(clusters.get(seq.get(i)), posI);
                    Translation2d posJ    = getSeqPos(seq, j - 1, clusters, robotPos);

                    double dOrig = posI.getDistance(nearestPoint(clusters.get(seq.get(i)), posI))
                                + exitIp1.getDistance(nearestPoint(clusters.get(seq.get(j)), posJ));

                    Translation2d nearJfromI = nearestPoint(clusters.get(seq.get(j)), posI);
                    Translation2d exitJfromI = exitPos(clusters.get(seq.get(j)), posI);
                    Translation2d nearIfromJ = nearestPoint(clusters.get(seq.get(i)), posI);

                    double dRev = posI.getDistance(nearJfromI)
                                + exitJfromI.getDistance(nearIfromJ);

                    if (dRev < dOrig - 0.05) {
                        List<Integer> sub = new ArrayList<>(seq.subList(i, j + 1));
                        java.util.Collections.reverse(sub);
                        for (int k = 0; k < sub.size(); k++) seq.set(i + k, sub.get(k));
                        improved = true;
                        break outer;
                    }
                }
            }
        }

        // ── Convert sequence → Pose2d targets ────────────────────────────────────
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
        for (Pose2d target : targets)
            sequence = sequence.andThen(
                AutoBuilder.pathfindToPose(target, CONSTRAINTS, passThroughVel));
        return sequence;
    }

    // ── Public command factories ──────────────────────────────────────────────────

    /**
     * @param budgetMeters   Max total path length in metres (e.g. 16.0)
     * @param passThroughVel Velocity m/s at each waypoint, 0.0 = full stop
     * @param maxClusters    Max clusters to visit, -1 = unlimited
     */
    public Command driveVelocityOpPath(double budgetMeters, double passThroughVel, int maxClusters) {
        return Commands.deferredProxy(() -> {
            Command c = buildVelocityOpPathCommand(budgetMeters, passThroughVel, maxClusters);
            return c != null ? c : Commands.none();
        });
    }

    // No-arg version uses class-level defaults
    public Command driveVelocityOpPath() {
        return driveVelocityOpPath(BUDGET_M, PASS_THROUGH_VEL, -1);
    }

    private static Translation2d getSeqPos(
            List<Integer> seq, int upToIndex,
            List<List<Translation2d>> clusters, Translation2d robot) {
        Translation2d cur = robot;
        for (int k = 0; k <= upToIndex && k < seq.size(); k++)
            cur = exitPos(clusters.get(seq.get(k)), cur);
        return cur;
    }

    public Command driveFastOpPath() {
        return Commands.deferredProxy(() -> {
            Command c = buildFastOpPathCommand();
            return c != null ? c : Commands.none();
        });
    }

    private Command buildMomentumPathCommand() {
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        Translation2d robotPos = swerve.getPose().getTranslation();
        List<List<Translation2d>> clusters = dbscan(balls);
        int n = clusters.size();

        List<Integer> seq = new ArrayList<>();
        boolean[] visited = new boolean[n];
        double spent = 0.0;
        Translation2d cur = robotPos;
        Double smoothH = null;

        boolean anyAdded = true;
        while (anyAdded) {
            anyAdded = false;
            double limit = MOMENTUM_MAX_TURN_RAD;

            for (int attempt = 0; attempt < 5; attempt++) {
                int bestIdx = -1;
                double bestScore = -1.0;
                double bestCost = 0.0;

                for (int i = 0; i < n; i++) {
                    if (visited[i])
                        continue;
                    List<Translation2d> c = clusters.get(i);

                    double cost = traverseDist(c, cur);
                    if (spent + cost > BUDGET_M)
                        continue;

                    Translation2d entry = nearestPoint(c, cur);
                    double turn = (smoothH != null)
                            ? headingDiff(smoothH, angleTo(cur, entry))
                            : 0.0;
                    if (turn > limit)
                        continue;

                    double score = (c.size() * c.size()) / Math.max(cost + MOMENTUM_HEAD_PEN * turn, 0.01);
                    if (score > bestScore) {
                        bestScore = score;
                        bestIdx = i;
                        bestCost = cost;
                    }
                }

                if (bestIdx >= 0) {
                    Translation2d ex = exitPos(clusters.get(bestIdx), cur);
                    double newH = angleTo(cur, ex);
                    if (smoothH == null) {
                        smoothH = newH;
                    } else {
                        double delta = ((newH - smoothH + Math.PI) % (2 * Math.PI)) - Math.PI;
                        smoothH += MOMENTUM_EMA_ALPHA * delta;
                    }
                    seq.add(bestIdx);
                    visited[bestIdx] = true;
                    spent += bestCost;
                    cur = ex;
                    anyAdded = true;
                    break; // restart outer loop
                }
                limit += MOMENTUM_RELAX_STEP;
            }
        }

        // ── Phase 2: Fill-in — insert skipped clusters between existing waypoints ─
        boolean changed = true;
        while (changed) {
            changed = false;

            // Build position list: pos[k] = robot exit after visiting seq[0..k-1]
            List<Translation2d> posList = buildPosList(seq, clusters, robotPos);

            int bestK = -1;
            int bestI = -1;
            double bestGain = -1.0;
            double bestExtra = 0.0;

            for (int i = 0; i < n; i++) {
                if (visited[i])
                    continue;
                List<Translation2d> c = clusters.get(i);

                for (int k = 0; k <= seq.size(); k++) {
                    Translation2d pb = posList.get(k);
                    Translation2d pa = (k < seq.size()) ? posList.get(k + 1) : posList.get(k);

                    Translation2d entry = nearestPoint(c, pb);
                    Translation2d ex = exitPos(c, pb);

                    double extra = (k < seq.size())
                            ? (pb.getDistance(entry) + entry.getDistance(ex) + ex.getDistance(pa)
                                    - pb.getDistance(pa))
                            : (pb.getDistance(entry) + entry.getDistance(ex));

                    if (spent + extra > BUDGET_M)
                        continue;

                    double hIn = angleTo(pb, entry);
                    double hPb = (k > 0) ? angleTo(posList.get(k - 1), pb) : hIn;
                    double hOut = (k < seq.size()) ? angleTo(ex, pa) : hIn;

                    if (headingDiff(hPb, hIn) > MOMENTUM_FILL_MAX_TURN)
                        continue;
                    if (headingDiff(hIn, hOut) > MOMENTUM_FILL_MAX_TURN)
                        continue;

                    double gain = (c.size() * c.size()) / Math.max(extra, 0.01);
                    if (gain > bestGain) {
                        bestGain = gain;
                        bestK = k;
                        bestI = i;
                        bestExtra = extra;
                    }
                }
            }

            if (bestI >= 0) {
                seq.add(bestK, bestI);
                visited[bestI] = true;
                spent += bestExtra;
                changed = true;
            }
        }

        // ── Phase 3: Append nearby stragglers at the end ──────────────────────────
        List<Translation2d> posList = buildPosList(seq, clusters, robotPos);
        cur = posList.get(posList.size() - 1);

        for (int i = 0; i < n; i++) {
            if (visited[i])
                continue;
            List<Translation2d> c = clusters.get(i);
            double cost = traverseDist(c, cur);
            if (cost > MOMENTUM_STRAGGLER_DIST)
                continue;
            if (spent + cost > BUDGET_M)
                continue;

            Translation2d entry = nearestPoint(c, cur);
            if (smoothH != null && headingDiff(smoothH, angleTo(cur, entry)) > Math.toRadians(150))
                continue;

            seq.add(i);
            visited[i] = true;
            spent += cost;
            cur = exitPos(c, cur);
        }

        // ── Convert sequence → Pose2d targets ────────────────────────────────────
        List<Pose2d> targets = new ArrayList<>();
        cur = robotPos;
        for (int idx : seq) {
            List<Pose2d> pts = clusterToSpinePoints(clusters.get(idx), cur);
            targets.addAll(pts);
            if (!pts.isEmpty())
                cur = pts.get(pts.size() - 1).getTranslation();
        }

        if (targets.isEmpty())
            return null;

        Logger.recordOutput("ObjectVision/MomentumWaypoints",
                targets.stream()
                        .map(p -> new Pose3d(p.getX(), p.getY(), 0.0, new Rotation3d()))
                        .toArray(Pose3d[]::new));

        Command sequence = Commands.none();
        for (Pose2d target : targets)
            sequence = sequence.andThen(
                    AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL));
        return sequence;
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

        Rotation2d heading = closest.minus(robotPos).getAngle();
        Pose2d target = new Pose2d(closest, heading);

        Logger.recordOutput("ObjectVision/ClosestBallTarget",
                new Pose3d(closest.getX(), closest.getY(), 0.0, new Rotation3d()));

        return AutoBuilder.pathfindToPose(target, CONSTRAINTS, 0.0);
    }

    public Command driveMomentumPath() {
        return Commands.deferredProxy(() -> {
            Command c = buildMomentumPathCommand();
            return c != null ? c : Commands.none();
        });
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
        return Math.abs(((h2 - h1 + Math.PI) % (2 * Math.PI)) - Math.PI);
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

    private static List<Translation2d> buildPosList(
            List<Integer> seq, List<List<Translation2d>> clusters, Translation2d robot) {
        List<Translation2d> pos = new ArrayList<>();
        pos.add(robot);
        for (int idx : seq)
            pos.add(exitPos(clusters.get(idx), pos.get(pos.size() - 1)));
        return pos;
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
            ballPosesBuf[i] = new Pose3d(rx[i], ry[i], 0.102, new Rotation3d());
        Logger.recordOutput("ObjectVision/Balls", java.util.Arrays.copyOf(ballPosesBuf, n));
    }

    private static List<Pose2d> clusterToSpinePoints(
            List<Translation2d> cluster, Translation2d from) {

        if (cluster.isEmpty())
            return List.of();

        if (cluster.size() == 1) {
            Translation2d p = cluster.get(0);
            return List.of(new Pose2d(p, p.minus(from).getAngle()));
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

        return List.of(new Pose2d(spineB, new Rotation2d(axX, axY)));
    }

    private Command buildDriveThroughClumpCommand() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null || rx.length == 0)
            return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

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
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        // Collect valid balls (not too close to the robot)
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        // Cluster and order greedily from the robot outward
        List<List<Translation2d>> clusters = dbscan(balls);
        List<Pose2d> targets = orderedClusterTargets(clusters, robotPos);

        return AutoBuilder.pathfindToPose(targets.get(0), CONSTRAINTS, 0.0);
    }

    private List<Translation2d> getValidBalls() {
        double[] rx = inputs.fuelX, ry = inputs.fuelY;
        if (rx == null || ry == null)
            return List.of();

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
        if (rx == null || ry == null || rx.length == 0)
            return null;

        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();
        double min2 = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        // Collect valid balls (not too close to the robot)
        List<Translation2d> balls = getValidBalls();
        if (balls.isEmpty())
            return null;

        // Cluster and order greedily from the robot outward
        List<List<Translation2d>> clusters = dbscan(balls);
        List<Pose2d> targets = orderedClusterTargets(clusters, robotPos);
        if (targets.isEmpty())
            return null;

        Logger.recordOutput("ObjectVision/Waypoints",
                targets.stream()
                        .map(p -> new Pose3d(p.getX(), p.getY(), 0, new Rotation3d()))
                        .toArray(Pose3d[]::new));

        Command sequence = Commands.none();
        for (Pose2d target : targets) {
            sequence = sequence.andThen(
                    AutoBuilder.pathfindToPose(target, CONSTRAINTS, PASS_THROUGH_VEL));
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
    // List<Translation2d> cluster, Translation2d from) {

    // if (cluster.isEmpty()) return List.of();

    // double sumX = 0, sumY = 0;
    // for (Translation2d p : cluster) { sumX += p.getX(); sumY += p.getY(); }
    // Translation2d centroid = new Translation2d(sumX / cluster.size(), sumY /
    // cluster.size());

    // Translation2d exit = cluster.stream()
    // .max((a, b) -> Double.compare(from.getDistance(a), from.getDistance(b)))
    // .get();

    // Rotation2d heading = exit.minus(centroid).getAngle();

    // List<Pose2d> points = new ArrayList<>();
    // points.add(new Pose2d(centroid, heading));
    // if (centroid.getDistance(exit) > 0.2)
    // points.add(new Pose2d(exit, heading));

    // return points;
    // }

    private static List<Pose2d> clusterToSmartPoints(
            List<Translation2d> cluster, Translation2d from) {

        if (cluster.isEmpty())
            return List.of();

        Translation2d exit = cluster.stream()
                .max((a, b) -> Double.compare(from.getDistance(a), from.getDistance(b)))
                .get();

        Rotation2d heading = exit.minus(from).getAngle();

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
                if (next.size() >= DBSCAN_MIN_PTS)
                    neighbors.addAll(next);
            }
            if (!cluster.contains(neighbor))
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