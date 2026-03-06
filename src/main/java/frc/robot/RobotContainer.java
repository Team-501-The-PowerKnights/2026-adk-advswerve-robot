// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
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
import frc.robot.subsystems.lift.Lift;
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
  private final Lift lift;
  private final Climber climber;
  /** */
  public final List<ISubsystem> subsystems;

  // Controllers
  private final CommandXboxController driverPad;
  private final CommandXboxController operPad;

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

    /*
     * Create all the subsystems based on whether enabled or not.
     */
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
      subsystems.add((ISubsystem) hopper);
    } else {
      hopper = null;
    }
    useSubsystem = SubsystemConstants.useIntake;
    useSubsystemTlmName = SubsystemConstants.intakeName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      intake = new Intake();
      subsystems.add((ISubsystem) intake);
    } else {
      intake = null;
    }
    useSubsystem = SubsystemConstants.useLift;
    useSubsystemTlmName = SubsystemConstants.liftName + "/useSubsystem";
    Logger.recordOutput(useSubsystemTlmName, useSubsystem);
    if (useSubsystem) {
      lift = new Lift();
      subsystems.add((ISubsystem) lift);
    } else {
      lift = null;
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
    /*
     * Create all the subsystems based on whether enabled or not.
     */
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    /*
     * Create the controllers and configure them.
     */
    driverPad = new CommandXboxController(0);
    operPad = new CommandXboxController(1);
    /*
     *
     */
    Logger.recordOutput("SubsystemDebug", SubsystemConstants.RUN_SUBSYSTEM_DEBUG);
    if (SubsystemConstants.RUN_SUBSYSTEM_DEBUG) {
      configureSubsystemDebugButtonBindings();
    } else {
      // Configure the button bindings
      configureButtonBindings();
    }

    /*
     * Create and set up the SysId functionality (if enabled).
     */
    // TODO: SysId routines
    if (SubsystemConstants.doSysId) {
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
    }
  }

  /**
   * Use this method to define your button->command mappings for the Subystem Debug mode. Every
   * subsystem has to have at least the manual control. By convention we are using the left joystick
   * of the Operator gamepad. Since only one subsystem should be enabled at a time this isn't a
   * problem.
   *
   * <p>Buttons can be created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  @SuppressWarnings("resource")
  private void configureSubsystemDebugButtonBindings() {

    int subsystemCount = 0;

    /*
     * Intake: Tied to left joystick Y axis of operator pad
     */
    if (SubsystemConstants.useIntake) {
      subsystemCount++;
      // Default command, manual control via triggers
      intake.setDefaultCommand(IntakeCommands.debugManual(intake, () -> -operPad.getLeftY()));
    }

    /*
     * Hopper: Tied to left joystick Y axis of operator pad
     */
    if (SubsystemConstants.useHopper) {
      subsystemCount++;
      // Default command, manual control via triggers
      hopper.setDefaultCommand(HopperCommands.debugManual(hopper, () -> -operPad.getLeftY()));
    }

    // How many subsystems were enabled? Is there a problem?
    if (subsystemCount == 0) {
      new Alert("No Subsystems enabled in DEBUG mode - Is this right?", AlertType.kWarning)
          .set(true);
    } else if (subsystemCount > 0) {
      new Alert(
              "Multiple Subsystems enabled in DEBUG mode (count = " + subsystemCount + ")",
              AlertType.kError)
          .set(true);
    }
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

    if (SubsystemConstants.useIntake) {}

    /*
     * Hopper: Operator Pad: X button - pull in, Y button push out
     */
    if (SubsystemConstants.useHopper) {
      hopper.setDefaultCommand(HopperCommands.stop(hopper));

      operPad.x().whileTrue(HopperCommands.pullIn(hopper));
      operPad.y().whileTrue(HopperCommands.pushOut(hopper));
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

  /***************************************************************************
   * Auto Delay Chooser Stuff
   ***************************************************************************/

  // Chooser for autonomous delay from Dashboard
  private SendableChooser<Integer> autoDelayChooser;
  // Delay that was selected
  private Integer autoDelaySelected;

  public void createAutoDelayChooser() {
    autoDelayChooser = new SendableChooser<>();

    // Default option is "no delay"
    autoDelayChooser.setDefaultOption("No Delay", Integer.valueOf(0));

    //
    autoDelayChooser.addOption("1 Sec", Integer.valueOf(1));
    autoDelayChooser.addOption("2 Sec", Integer.valueOf(2));
    autoDelayChooser.addOption("3 Sec", Integer.valueOf(3));
    autoDelayChooser.addOption("4 Sec", Integer.valueOf(4));
    autoDelayChooser.addOption("5 Sec", Integer.valueOf(5));
    autoDelayChooser.addOption("6 Sec", Integer.valueOf(6));
    autoDelayChooser.addOption("7 Sec", Integer.valueOf(7));
    autoDelayChooser.addOption("8 Sec", Integer.valueOf(8));
    autoDelayChooser.addOption("9 Sec", Integer.valueOf(9));
    autoDelayChooser.addOption("10 Sec", Integer.valueOf(10));
    autoDelayChooser.addOption("11 Sec", Integer.valueOf(11));
    autoDelayChooser.addOption("12 Sec", Integer.valueOf(12));
    autoDelayChooser.addOption("13 Sec", Integer.valueOf(13));
    autoDelayChooser.addOption("14 Sec", Integer.valueOf(14));

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Delay Chooser", autoDelayChooser);
  }

  public Integer getAutonomousDelay() {
    autoDelaySelected = autoDelayChooser.getSelected();
    return autoDelaySelected;
  }

  private class DelayAutoCommand extends Command {
    /** The timer used for waiting. */
    protected Timer m_timer = new Timer();

    private double m_duration;

    public DelayAutoCommand() {}

    @Override
    public void initialize() {
      m_duration = getAutonomousDelay().doubleValue();
      m_timer.restart();
      System.out.println("AutoDelayCommand initialized");
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
      System.out.println("AutoDelayCommand done");
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("duration", () -> m_duration, null);
    }
  }

  /***************************************************************************
   * Path Planner Stuff
   ***************************************************************************/

  void configurePathPlannerCommands() {
    //
    NamedCommands.registerCommand("Delay Auto Start", Commands.sequence(new DelayAutoCommand()));

    //
    // NamedCommands.registerCommand(
    //     "Release Climber Latch",
    //     Commands.sequence(
    //         ClimberCommands.unlatch(climber), new WaitCommand(0.5),
    // ClimberCommands.stop(climber)));
  }
}
