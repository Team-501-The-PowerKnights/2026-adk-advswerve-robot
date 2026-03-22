package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;
import java.util.function.DoubleSupplier;

public class LauncherCommands {

  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  /** Private constructor so can't be instantiated externally */
  private LauncherCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * @param hopper
   * @return
   */
  public static Command stop(Launcher launcher) {
    return launcher.runOnce(launcher::stop).withName("LauncherStop");
  }

  public static Command SetIdle(Launcher launcher) {
    return launcher.runOnce(launcher::setIdle).withName("LauncherSetIdle");
  }

  public static Command manual(Launcher launcher, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          launcher.acceptInput(speed);
        },
        launcher);
  }

  /**
   * @param launcher
   * @return
   */
  public static Command pullIn(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(-LauncherConstants.defaultSpeed), launcher::stop)
        .withName("LauncherPullIn");
  }
  /**
   * @param launcher
   * @return
   */
  public static Command pullInNear(Launcher launcher) {
    return launcher
        .runEnd(
            () -> launcher.acceptInput(-LauncherConstants.kLauncherNearSpeed), launcher::setIdle)
        .withName("LauncherPullIn");
  }
  /**
   * @param launcher
   * @return
   */
  public static Command pullInMid(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(-LauncherConstants.kLauncherMidSpeed), launcher::setIdle)
        .withName("LauncherPullIn");
  }
  /**
   * @param launcher
   * @return
   */
  public static Command pullInFar(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(-LauncherConstants.kLauncherFarSpeed), launcher::setIdle)
        .withName("LauncherPullIn");
  }

  /**
   * @param launcher
   * @return
   */
  public static Command pushOut(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(+LauncherConstants.defaultSpeed), launcher::stop)
        .withName("LauncherPushOut");
  }

  /**
   * Command to do a manual control of the subystem for debugging.
   *
   * <p>Every subsystem <b>has</b> to have this to allow debug.
   *
   * @param launcher
   * @param speedSupplier
   * @return
   */
  public static Command debugManual(Launcher launcher, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          launcher.acceptInput(speed);
        },
        launcher);
  }
}
