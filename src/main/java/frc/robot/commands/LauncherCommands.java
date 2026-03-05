package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.launcher.Launcher;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
  public static Command joystickDrive(
      Launcher launcher, DoubleSupplier leftJoystick, Hopper hopper) {

    return new RunCommand(
        () -> {
          double left = leftJoystick.getAsDouble(); // -1..1

          if (left > 0.05 || left < -0.05) {
            launcher.setPercent(left);
            hopper.acceptTeleopInput(-left);

          } else {
            launcher.stop();
            hopper.acceptTeleopInput(0.0);
          }
        },
        launcher);
  }
}
