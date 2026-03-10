package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;
import java.util.function.DoubleSupplier;

public class LauncherCommands {

  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  public static Command joystickDrive(Launcher launcher, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          launcher.acceptInput(speed);
        },
        launcher);
  }

  public static Command stop(Launcher launcher) {
    return launcher.runOnce(launcher::stop).withName("LauncherStop");
  }

  /**
   * @param launcher
   * @return
   */
  public static Command LaunchIn(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(-LauncherConstants.defaultSpeed), launcher::stop)
        .withName("LauncherPullIn");
  }

  /**
   * @param launcher
   * @return
   */
  public static Command launchOut(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(+LauncherConstants.defaultSpeed), launcher::launcherIdle)
        .withName("LauncherIdle");
  }

  public static Command launchIdle(Launcher launcher) {
    return launcher
        .runEnd(() -> launcher.acceptInput(+LauncherConstants.defaultIdleSpeed), launcher::stop)
        .withName("LauncherIdle");
  }

  public static Command joystickDrive2(Launcher launcher, DoubleSupplier leftJoystick) {

    return new RunCommand(
        () -> {
          double left = leftJoystick.getAsDouble(); // -1..1

          if (left > DEADBAND || left < -DEADBAND) {
            launcher.acceptInput(left);
          } else {
            launcher.stop();
          }
        },
        launcher);
  }

  public static Command debugManual(Launcher launcher, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          launcher.acceptInput(speed);
        },
        launcher);
  }
}
