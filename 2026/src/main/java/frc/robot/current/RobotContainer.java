// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.subsystems.Climber;
import frc.robot.current.subsystems.Hopper;
import frc.robot.current.subsystems.Intake;
import frc.robot.current.subsystems.LedOperation;
import frc.robot.current.subsystems.Outtake;
import frc.robot.current.Pather.Direction;
import frc.robot.current.subsystems.Pivot;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.GyroIO;
import frc.robot.current.subsystems.swerveDrive.GyroIONavX;
import frc.robot.current.subsystems.swerveDrive.ModuleIO;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSim;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSpark;
import frc.robot.lib.ObjectVision.ObjectVision;
import frc.robot.lib.ObjectVision.ObjectVisionIODetection;
import frc.robot.lib.commands.DriveCommands;
import frc.robot.lib.util.AllianceRotationUtil;
import frc.robot.lib.vision.Vision;
import frc.robot.lib.vision.VisionIO;
import frc.robot.lib.vision.VisionIOPhotonVision;
import frc.robot.lib.vision.VisionIOPhotonVisionSim;
import static frc.robot.lib.vision.VisionConstants.*;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private Drive drive;
  private Intake intake;
  private Pivot pivot;
  @SuppressWarnings("unused")
  private Vision vision;
  private ObjectVision objectVision;
  private Outtake outtake;
  private Hopper hopper;
  private Climber climber;
  @SuppressWarnings("unused")
  private LedOperation leds;

  private static final ControlType controlType = ControlType.ONEXBOX;

  public enum ControlType {
    ONEXBOX, TWOXBOX
  }

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController controlXbox = new CommandXboxController(OperatorConstants.kOtherControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // exPivot = new ExamplePivot(Constants.robot);
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

        vision = new Vision(drive::addVisionMeasurement,
            new VisionIOPhotonVision(camera0Name, robotToCamera0),
            new VisionIOPhotonVision(camera1Name, robotToCamera1),
            new VisionIOPhotonVision(camera2Name, robotToCamera2),
            new VisionIOPhotonVision(camera3Name, robotToCamera3));
        break;

      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        vision = new Vision(drive::addVisionMeasurement,
            // new VisionIOPhotonVision(camera0Name, robotToCamera0),
            new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
            // new VisionIOPhotonVision(camera2Name, robotToCamera2),
            new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));
        break;
      default:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });

        vision = new Vision(drive::addVisionMeasurement,
            new VisionIO() {
            },
            new VisionIO() {
            });
        break;
    }

    hopper = new Hopper();
    objectVision = new ObjectVision(drive, new ObjectVisionIODetection());
    outtake = new Outtake(drive, hopper);
    intake = new Intake(drive);
    pivot = new Pivot();
    climber = new Climber();

    leds = new LedOperation(outtake, intake, climber);

    NamedCommands.registerCommand("Launch", outtake.timedLaunch(6));
    NamedCommands.registerCommand("LaunchOff", outtake.stop());
    NamedCommands.registerCommand("IntakeOn", intake.intakeSlow());
    NamedCommands.registerCommand("IntakeOff", intake.stop());
    NamedCommands.registerCommand("PivotDown", pivot.gotoCollectionPos());
    NamedCommands.registerCommand("PivotUp", pivot.gotoStoredPos());
    NamedCommands.registerCommand("ClimbUp", climber.climbMaxBoth());
    NamedCommands.registerCommand("ClimbDown", climber.climbMinBoth());

    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

    // Add sysID routines to the SendableChooser for autos
    if (Constants.isTuningMode) {
      autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("FFCharacterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    }

    SmartDashboard.putData("KindleCommand", objectVision.kindleCommand());
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Point To Hub", DriveCommands.joystickDrivePointToTarget(
        drive,
        () -> -driveXbox.getLeftY(),
        () -> -driveXbox.getLeftX(),
        // compute absolute heading to the target (field frame) from current robot pose
        () -> {
          Pose2d target = AllianceRotationUtil.apply(FieldConstants.Elements.blueHubPose);
          Pose2d robotPose = drive.getPose();
          double dx = target.getTranslation().getX() - robotPose.getTranslation().getX();
          double dy = target.getTranslation().getY() - robotPose.getTranslation().getY();
          return Math.atan2(dy, dx);
        }));

    SmartDashboard.putData("Slow Mode", DriveCommands.joystickDrive(
        drive,
        () -> -0.45 * driveXbox.getLeftY(),
        () -> -0.45 * driveXbox.getLeftX(),
        () -> -0.5 * driveXbox.getRightX()));

    Pather.configureKindleListeners();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveXbox.getLeftY(),
            () -> -driveXbox.getLeftX(),
            () ->  -0.75 * driveXbox.getRightX()));

    // driveXbox.y()
    //     .whileTrue(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -0.45 * driveXbox.getLeftY(),
    //             () -> -0.45 * driveXbox.getLeftX(),
    //             () -> -0.5 * driveXbox.getRightX()));

    // // Lock to 0° when A button is held
    // driveXbox
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driveXbox.getLeftY(),
    //             () -> -driveXbox.getLeftX(),
    //             () -> Rotation2d.kCCW_90deg));

    driveXbox.back().whileTrue(
        Commands.defer(() -> Pather.trenchAlign(Direction.LEFT), Set.of(drive)));
    driveXbox.start().whileTrue(
       Commands.defer(() -> Pather.trenchAlign(Direction.RIGHT), Set.of(drive)));

    // driveXbox.leftBumper().whileTrue(
    //     Commands.defer(() -> Pather.pathFinderPro(Pather.Target.HUBSHOOTLEFT), Set.of(drive)));
    // driveXbox.rightBumper().whileTrue(
    //    Commands.defer(() -> Pather.pathFinderPro(Pather.Target.HUBSHOOTRIGHT), Set.of(drive)));
    // driveXbox.povRight().whileTrue(
    //     Commands.defer(() -> Pather.pathFinder(Pather.Target.OUTPOST, null), Set.of(drive)));

    // Switch to X pattern when X button is pressed
    driveXbox.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driveXbox
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
                .ignoringDisable(true));

    switch (controlType) {
      case ONEXBOX:
        driveXbox.leftTrigger().whileTrue(intake.intakeSlow()).onFalse(intake.stop());
        driveXbox.leftBumper().whileTrue(intake.intakeFast()).onFalse(intake.stop());
        // driveXbox.rightTrigger().onTrue(outtake.variableLaunchMap()).onFalse(outtake.stop());
        driveXbox.rightTrigger().onTrue(outtake.launcherPro()).onFalse(outtake.stop());

        driveXbox.povUp().onTrue(pivot.gotoStoredPos());
        driveXbox.povDown().onTrue(pivot.gotoCollectionPos());

        driveXbox.y().onTrue(climber.climbUp()).onFalse(climber.stop());
        driveXbox.a().onTrue(climber.climbDown()).onFalse(climber.stop());
        break;

      case TWOXBOX:
      default:
        driveXbox.leftBumper().whileTrue(
            DriveCommands.joystickDrivePointToTarget(
                drive,
                () -> -driveXbox.getLeftY(),
                () -> -driveXbox.getLeftX(),
                // compute absolute heading to the target (field frame) from current robot pose
                () -> {
                  Pose2d target = AllianceRotationUtil.apply(FieldConstants.Elements.blueHubPose);
                  Pose2d robotPose = drive.getPose();
                  double dx = target.getTranslation().getX() - robotPose.getTranslation().getX();
                  double dy = target.getTranslation().getY() - robotPose.getTranslation().getY();
                  return Math.atan2(dy, dx);
                }));

        controlXbox.rightBumper().onTrue(outtake.continuousLaunch()).onFalse(outtake.stop());

        controlXbox.povUp().onTrue(pivot.gotoStoredPos());
        controlXbox.povDown().onTrue(pivot.gotoCollectionPos());

        controlXbox.leftTrigger().whileTrue(intake.intakeSlow()).onFalse(intake.stop());
        // controlXbox.leftBumper().onTrue(intake.intakeFast()).onFalse(intake.stop());

        driveXbox.povLeft().whileTrue(objectVision.kindleCommand());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDriveSubsystem() {
    return drive;
  }
}
