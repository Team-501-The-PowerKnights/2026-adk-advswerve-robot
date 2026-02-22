// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.SubsystemConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakelift.IntakeLift;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final Shooter shooter;
  private final Turret turret;
  private final Hopper hopper;
  private final Intake intake;
  private final IntakeLift intakeLift;
  private final Climber climber;
  /** */
  public final List<ISubsystem> subsystems;

  // Controller
  private final CommandXboxController driverPad = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    subsystems = new ArrayList<ISubsystem>();
    boolean useSubsystem;
    String useSubsystemTlmName;

    useSubsystem = SubsystemConstants.useVision;
    useSubsystemTlmName = SubsystemConstants.visionName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      vision = new Vision();
      subsystems.add(vision);
    } else {
      vision = null;
    }
    useSubsystem = SubsystemConstants.useShooter;
    useSubsystemTlmName = SubsystemConstants.shooterName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      shooter = new Shooter();
      subsystems.add(shooter);
    } else {
      shooter = null;
    }
    useSubsystem = SubsystemConstants.useTurret;
    useSubsystemTlmName = SubsystemConstants.turretName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      turret = new Turret();
      subsystems.add(turret);
    } else {
      turret = null;
    }
    useSubsystem = SubsystemConstants.useHopper;
    useSubsystemTlmName = SubsystemConstants.hopperName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      hopper = new Hopper();
      subsystems.add(hopper);
    } else {
      hopper = null;
    }
    useSubsystem = SubsystemConstants.useIntake;
    useSubsystemTlmName = SubsystemConstants.intakeName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      intake = new Intake();
      subsystems.add(intake);
    } else {
      intake = null;
    }
    useSubsystem = SubsystemConstants.useIntakeLift;
    useSubsystemTlmName = SubsystemConstants.intakeLiftName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      intakeLift = new IntakeLift();
      subsystems.add(intakeLift);
    } else {
      intakeLift = null;
    }
    useSubsystem = SubsystemConstants.useClimber;
    useSubsystemTlmName = SubsystemConstants.climberName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      climber = new Climber();
      subsystems.add(climber);
    } else {
      climber = null;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverPad.getLeftY(),
            () -> -driverPad.getLeftX(),
            () -> -driverPad.getRightX()));

    // Lock to 0° when A button is held
    driverPad
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverPad.getLeftY(),
                () -> -driverPad.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverPad.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverPad
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    if (SubsystemConstants.useTurret) {
      turret.setDefaultCommand(
          TurretCommands.manual(
              turret, () -> (driverPad.getLeftTriggerAxis() + -driverPad.getRightTriggerAxis())));
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
}
