package frc.robot.lib.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.current.Constants;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.lib.util.GeomUtil;

public class DriveToPose extends Command {
    private final Drive drive;
    private final boolean slowMode;
    private final Supplier<Pose2d> poseSupplier;

    private boolean running = false;
    private final ProfiledPIDController driveController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private Translation2d lastSetpointTranslation;

    private static double driveKp;
    private static double driveKd;
    private static double thetaKp;
    private static double thetaKd;
    private static double driveMaxVelocity;
    private static double driveMaxVelocitySlow;
    private static double driveMaxAcceleration;
    private static double thetaMaxVelocity;
    private static double thetaMaxVelocitySlow;
    private static double thetaMaxAcceleration;
    private static double driveTolerance;
    private static double driveToleranceSlow;
    private static double thetaTolerance;
    private static double thetaToleranceSlow;
    private static double ffMinRadius;
    private static double ffMaxRadius;

    static {
        switch (Constants.robot) {
            case "SIM":
            case "Real":
            default:
                driveKp = 6.0; // Old value = 6
                driveKd = 0.0;
                thetaKp = 5.0;
                thetaKd = 0.0;
                driveMaxVelocity = Units.feetToMeters(15.3); // tested max
                driveMaxVelocitySlow = Units.feetToMeters(2); // "slow" they said
                driveMaxAcceleration = Units.feetToMeters(10); // 12 works well under a charged-ish battery
                thetaMaxVelocity = Units.degreesToRadians(360);
                thetaMaxVelocitySlow = Units.degreesToRadians(90);
                thetaMaxAcceleration = Units.degreesToRadians(360); // keep this low for brownout purposes
                driveTolerance = 0.05; // Old value = 0.01
                driveToleranceSlow = 0.06;
                thetaTolerance = Units.degreesToRadians(1);
                thetaToleranceSlow = Units.degreesToRadians(1);
                ffMinRadius = 0.2*4;
                ffMaxRadius = 0.8*4;
                break;
        }
    }

    public DriveToPose(Drive drive, boolean slowMode, Pose2d pose) {
        this(drive, slowMode, () -> pose);
    }

    public DriveToPose(Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier) {
        this.drive = drive;
        this.slowMode = slowMode;
        this.poseSupplier = poseSupplier;
        addRequirements(drive);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Reset all controllers
        var currentPose = drive.getPose();
        driveController.reset(
            currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
            Math.min(
                0.0,
                -new Translation2d(drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
                    .rotateBy(poseSupplier.get().getTranslation().minus(drive.getPose().getTranslation()).getAngle().unaryMinus())
                    .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(), drive.getFieldVelocity().omegaRadiansPerSecond);
        lastSetpointTranslation = drive.getPose().getTranslation();
    }

    @Override
    public void execute() {
        running = true;

        // Update constants
        driveController.setP(driveKp);
        driveController.setD(driveKd);
        driveController.setConstraints(new TrapezoidProfile.Constraints(slowMode ? driveMaxVelocitySlow : driveMaxVelocity, driveMaxAcceleration));
        driveController.setTolerance(slowMode ? driveToleranceSlow : driveTolerance);
        thetaController.setP(thetaKp);
        thetaController.setD(thetaKd);
        thetaController.setConstraints(new TrapezoidProfile.Constraints(slowMode ? thetaMaxVelocitySlow : thetaMaxVelocity, thetaMaxAcceleration));
        thetaController.setTolerance(slowMode ? thetaToleranceSlow : thetaTolerance);

        // Get current and target pose
        var currentPose = drive.getPose();
        var targetPose = poseSupplier.get();

        // Calculate drive speed
        double currentDistance = 
            currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
        double ffScalar = 
            MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
            lastSetpointTranslation.getDistance(targetPose.getTranslation()),
            driveController.getSetpoint().velocity);
        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScalar
            + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
        lastSetpointTranslation =
            new Pose2d(
                    targetPose.getTranslation(),
                    currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                    GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
                .getTranslation();
        
        // Calculate theta speed
        double thetaVelocity =
            thetaController.getSetpoint().velocity * ffScalar
                + thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs =
            Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        // Command speeds
        var driveVelocty =
            new Pose2d(
                    new Translation2d(),
                    currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocty.getX(), driveVelocty.getY(), thetaVelocity, currentPose.getRotation()));

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput("Odometry/DriveToPoseSetpoint", 
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
        Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
        drive.stop();
        Logger.recordOutput("Odometry/DriveToPoseSetpoint", new double[] {});
        Logger.recordOutput("Odometry/DriveToPoseGoal", new double[] {});
    }

    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
            && Math.abs(driveErrorAbs) < driveTolerance
            && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }

    /** Returns whether the command is actively running. */
    public boolean isRunning() {
        return running;
    }
}
