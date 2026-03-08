package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.launcher.Launcher;
import java.util.function.DoubleSupplier;

public class LauncherCommands {

  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  public static Command joystickDrive(
      Launcher launcher, DoubleSupplier leftJoystick, Hopper hopper) {

    return new RunCommand(
        () -> {
          double left = leftJoystick.getAsDouble(); // -1..1

          if (left > 0.05 || left < -0.05) {
            launcher.setPercent(left);
            // hopper.acceptInput(-left);
          } else {
            launcher.stop();
            // hopper.stop();
          }
        },
        launcher);
  }

  public static Command joystickDrive(Launcher launcher, DoubleSupplier leftJoystick) {

    return new RunCommand(
        () -> {
          double left = leftJoystick.getAsDouble(); // -1..1

          if (left > 0.05 || left < -0.05) {
            launcher.setPercent(left);
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
          launcher.setPercent(speed);
        },
        launcher);
  }
}
