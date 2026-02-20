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
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenDriveSparkTurn;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Turret turret;
  private final Hopper hopper;
  private final Feeder feeder;
  private final Lift lift;
  private final Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

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
                new ModuleIOKrakenDriveSparkTurn(0),
                new ModuleIOKrakenDriveSparkTurn(1),
                new ModuleIOKrakenDriveSparkTurn(2),
                new ModuleIOKrakenDriveSparkTurn(3));
        if (Constants.ENABLE_VISION) {
          // Limelight table name is commonly "limelight" or "limelight-<name>"
          vision = new Vision(new VisionIOLimelight("limelight"));
        } else {
          vision = null; // new Vision(new VisionIO() {});
        }
        if (Constants.ENABLE_SHOOTER) {
          shooter = new Shooter();
          feeder = new Feeder();
        } else {
          shooter = null;
          feeder = null;
        }
        if (Constants.ENABLE_TURRET) {
          turret = new Turret();
        } else {
          turret = null;
        }
        if (Constants.ENABLE_HOPPER) {
          hopper = new Hopper();
        } else {
          hopper = null;
        }
        if (Constants.ENABLE_LIFT) {
          lift = new Lift();
        } else {
          lift = null;
        }
        if (Constants.ENABLE_INTAKE) {
          intake = new Intake();
        } else {
          intake = null;
        }
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
        // No Limelight in sim (unless you build a sim IO later)
        vision = new Vision(new VisionIO() {});
        shooter = null;
        turret = null;
        hopper = null;
        feeder = null;
        intake = null;
        lift = null;
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
        // Replayed: disable vision IO
        vision = new Vision(new VisionIO() {});
        shooter = null;
        turret = null;
        hopper = null;
        feeder = null;
        intake = null;
        lift = null;
        break;
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
    // if (Constants.ENABLE_LIFT && Constants.ENABLE_INTAKE) {
    //   intake.setDefaultCommand(IntakeCommands.autoRun(intake, lift));
    // }
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  // MARK: - BUTTONS
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    // Shooter manual controls, if enabled
    if (Constants.ENABLE_SHOOTER) {
      shooter.setDefaultCommand(
          ShooterCommands.triggerDrive(
              shooter, feeder, controller::getLeftTriggerAxis, controller::getRightTriggerAxis));
    }
    // Turret manual controls, if enabled
    if (Constants.ENABLE_TURRET) {
      controller.leftBumper().whileTrue(Commands.startEnd(turret::turnLeft, turret::stop, turret));
      controller
          .rightBumper()
          .whileTrue(Commands.startEnd(turret::turnRight, turret::stop, turret));
    }
    // Hopper manual controls, if enabled
    if (Constants.ENABLE_HOPPER) {
      operator.x().whileTrue(HopperCommands.forward(hopper));
      operator.y().whileTrue(HopperCommands.reverse(hopper));
    }
    if (Constants.ENABLE_LIFT) {
      // A button → toggle lift + intake state
      operator.a().onTrue(SuperstructureCommands.toggleLiftAndIntake(lift, intake));

      // Hold B → reverse intake
      operator.b().whileTrue(IntakeCommands.reverse(intake));
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

  /** Call this once per loop from Robot.robotPeriodic() to fuse vision into odometry. */
  public void robotPeriodic() {
    if (Constants.ENABLE_VISION) {
      vision
          .getMeasurement()
          .ifPresent(
              meas -> {
                // Optional sanity gate: ignore huge jumps
                Pose2d current = drive.getPose();
                double dx = meas.pose().getX() - current.getX();
                double dy = meas.pose().getY() - current.getY();
                double dist = Math.hypot(dx, dy);

                if (dist < 2.5) { // tune if needed
                  drive.addVisionMeasurement(meas.pose(), meas.timestampSec(), meas.stdDevs());
                }
              });
    }
  }
}
