// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.current.Constants.OperatorConstants;
import frc.robot.current.subsystems.ExamplePivot;
import frc.robot.current.subsystems.Intake;
import frc.robot.current.subsystems.LedOperation;
import frc.robot.current.subsystems.Outtake;
import frc.robot.current.subsystems.PathFollower;
import frc.robot.current.subsystems.Pivot;
import frc.robot.current.subsystems.Hopper;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.GyroIONavX;
import frc.robot.current.subsystems.swerveDrive.ModuleIOSpark;
import frc.robot.lib.commands.DriveCommands;
import frc.robot.lib.vision.VisionIOPhotonVision;
import frc.robot.lib.vision.Vision;
import static frc.robot.lib.vision.VisionConstants.*;

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
  private LedOperation leds;
  private Intake intake;
  private Pivot pivot;
  private Vision vision;
  private Outtake outtake;

  private static Boolean cameraYes = true;
  private PathFollower pathFollower;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController controlXbox = new CommandXboxController(OperatorConstants.kOtherControllerPort);

  private final SendableChooser<Command> autoChooser;
  private Command autoDefault = Commands.print("Default auto selected. No autonomous command configured.");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // leds = new LedOperation();
    // exPivot = new ExamplePivot(Constants.robot);

    drive = new Drive(
        new GyroIONavX(),
        new ModuleIOSpark(0),
        new ModuleIOSpark(1),
        new ModuleIOSpark(2),
        new ModuleIOSpark(3));

    if (cameraYes == true) {
      vision = new Vision(drive::addVisionMeasurement,
          // new VisionIOPhotonVision(camera0Name, robotToCamera0),
          new VisionIOPhotonVision(camera1Name, robotToCamera1),
          // new VisionIOPhotonVision(camera2Name, robotToCamera2),
          new VisionIOPhotonVision(camera3Name, robotToCamera3));
    }

    Hopper hopper = new Hopper();
    pathFollower = new PathFollower(drive);
    outtake = new Outtake(drive, hopper);
    intake = new Intake(drive);
    pivot = new Pivot();

    NamedCommands.registerCommand("Launch", outtake.timedLaunch(8));
    NamedCommands.registerCommand("IntakeOn", intake.intake());
    NamedCommands.registerCommand("IntakeOff", intake.stop());
    NamedCommands.registerCommand("PivotDown", pivot.gotoCollectionPos());
    NamedCommands.registerCommand("PivotUp", pivot.gotoStoredPos());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Launch Only",
        Commands.sequence(
            pivot.gotoCollectionPos(),
            outtake.timedLaunch(8)));

    SmartDashboard.putData("Auto Chooser", autoChooser);

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

    driveXbox.back()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -0.45 * driveXbox.getLeftY(),
                () -> -0.45 * driveXbox.getLeftX(),
                () -> -0.5 * driveXbox.getRightX()));

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

    driveXbox.start().whileTrue(Commands.run(() -> pathFollower.driveThruTrench()))
        .onFalse(Commands.runOnce(() -> {
          drive.stop();
        }, drive));

    driveXbox.start().whileTrue(Commands.run(() -> AutoBuilder.buildAuto("Go To Outpost")));

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

    controlXbox.rightBumper().onTrue(outtake.continuousLaunch()).onFalse(outtake.stop());

    controlXbox.rightTrigger().onTrue(outtake.variableLaunchEquation()).onFalse(outtake.stop());

    controlXbox.povUp().onTrue(pivot.gotoStoredPos());
    controlXbox.povDown().onTrue(pivot.gotoCollectionPos());

    controlXbox.leftTrigger().whileTrue(intake.intake()).onFalse(intake.stop());
    controlXbox.leftBumper().onTrue(intake.spit());

    // exPivot.setDefaultCommand(
    // Commands.run(() -> {
    // exPivot.adjustHeight(-1 * controlXbox.getLeftY());
    // },
    // exPivot));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // controlXbox.a().whileTrue(intake.intake()).onFalse(intake.stop());
    // controlXbox.x().onTrue(outtake.launch());

    // swerveDrive.setDefaultCommand(
    // new DriveWithController(swerveDrive, 0.5, 0.5, () -> driveXbox.getLeftX(), ()
    // -> driveXbox.getLeftY(),
    // () -> driveXbox.getRightX(), () -> driveXbox.getRightY(), () ->
    // driveXbox.a().getAsBoolean()));
    // driveXbox.x().onTrue(Commands.runOnce(swerveDrive::stopWithX, swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return AutoBuilder.buildAuto("Straight Back");

    // if (autoChooser.get() != null) {
    // System.out.print("Auto Path " + autoChooser.get().getName());
    return autoChooser.getSelected();
    // } else {
    // return Commands.print("No autonomous command configured");
    // }
  }
}
