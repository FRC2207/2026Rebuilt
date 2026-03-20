// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.subsystems.Intake;
import frc.robot.current.subsystems.Outtake;
import frc.robot.lib.commands.PathFollower;
//import frc.robot.current.subsystems.PathFollower;
import frc.robot.current.subsystems.Pivot;
import frc.robot.current.subsystems.Hopper;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.GyroIO;
import frc.robot.current.subsystems.swerveDrive.GyroIONavX;
import frc.robot.current.subsystems.swerveDrive.ModuleIO;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSim;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSpark;

import frc.robot.lib.commands.DriveCommands;
import frc.robot.lib.vision.VisionIOPhotonVision;
import frc.robot.lib.vision.VisionIOPhotonVisionSim;
import frc.robot.lib.vision.Vision;
import frc.robot.lib.vision.VisionIO;

import static frc.robot.lib.vision.VisionConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  // private ExamplePivot exPivot;
  // private SwerveDrive swerveDrive;
  private Drive drive;
  private Intake intake;
  private Pivot pivot;
  private Vision vision;
  private Outtake outtake;
  private Hopper hopper;

  // private PathFollower pathFollower;

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

    // leds = new LedOperation();
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
          // new VisionIOPhotonVision(camera0Name, robotToCamera0),
          new VisionIOPhotonVision(camera1Name, robotToCamera1),
          // new VisionIOPhotonVision(camera2Name, robotToCamera2),
          new VisionIOPhotonVision(camera3Name, robotToCamera3));
        break;
      
      case SIM:
        drive = 
          new Drive(
            new GyroIO() {},
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
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        
        vision = new Vision(drive::addVisionMeasurement,
          // new VisionIOPhotonVision(camera0Name, robotToCamera0),
          new VisionIO() {},
          // new VisionIOPhotonVision(camera2Name, robotToCamera2),
          new VisionIO() {});
        break;
    }


    hopper = new Hopper();
    //pathFollower = new PathFollower(drive);
    outtake = new Outtake(drive, hopper);
    intake = new Intake(drive);
    pivot = new Pivot();

    NamedCommands.registerCommand("Launch", outtake.timedLaunch(8));
    NamedCommands.registerCommand("IntakeOn", intake.intake());
    NamedCommands.registerCommand("IntakeOff", intake.stop());
    NamedCommands.registerCommand("PivotDown", pivot.gotoCollectionPos());
    NamedCommands.registerCommand("PivotUp", pivot.gotoStoredPos());

    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

    // Add autonomous routines to the SendableChooser
    // autoDefault = Commands.none();
    // autoChooser.addDefaultOption("Default Auto", autoDefault);

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
    // Configure the trigger bindings
    configureBindings();
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
            () -> -driveXbox.getRightX()));

    // driveXbox.back()
    //     .whileTrue(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -0.45 * driveXbox.getLeftY(),
    //             () -> -0.45 * driveXbox.getLeftX(),
    //             () -> -0.5 * driveXbox.getRightX()));

    // Lock to 0° when A button is held
    driveXbox
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveXbox.getLeftY(),
                () -> -driveXbox.getLeftX(),
                () -> Rotation2d.kCCW_90deg));

    driveXbox.leftBumper().whileTrue(
        DriveCommands.joystickDrivePointTarget(
            drive,
            () -> -driveXbox.getLeftY(),
            FieldConstants.Elements.blueHubPose));

    driveXbox.start().whileTrue(new PathFollower(drive, PathFollower.Target.TRENCH));
    //driveXbox.back().whileTrue(new PathFollower(drive, PathFollower.Target.OUTPOST));
    driveXbox.rightBumper().whileTrue(new PathFollower(drive, PathFollower.Target.HUBSHOOT));

        // .onFalse(Commands.runOnce(() -> {
        //   drive.stop();
        // }, drive));

    //driveXbox.back().whileTrue(Commands.run(() -> pathFollower.driveToOutpost()));

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
        //driveXbox.rightBumper().onTrue(outtake.continuousLaunch()).onFalse(outtake.stop());

        driveXbox.rightTrigger().onTrue(outtake.variableLaunchEquation()).onFalse(outtake.stop());

        driveXbox.povUp().onTrue(pivot.gotoStoredPos());
        driveXbox.povDown().onTrue(pivot.gotoCollectionPos());

        driveXbox.leftTrigger().whileTrue(intake.intake()).onFalse(intake.stop());
        driveXbox.leftBumper().onTrue(intake.spit());
        break;
      case TWOXBOX:
      default:
        controlXbox.rightBumper().onTrue(outtake.continuousLaunch()).onFalse(outtake.stop());

        controlXbox.rightTrigger().onTrue(outtake.variableLaunchEquation()).onFalse(outtake.stop());

        controlXbox.povUp().onTrue(pivot.gotoStoredPos());
        controlXbox.povDown().onTrue(pivot.gotoCollectionPos());

        controlXbox.leftTrigger().whileTrue(intake.intake()).onFalse(intake.stop());
        controlXbox.leftBumper().onTrue(intake.spit());
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
