package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.launcher.Launcher;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
  public static Command triggerDrive(Launcher launcher, DoubleSupplier leftTrigger) {

    return new RunCommand(
        () -> {
          double left = leftTrigger.getAsDouble(); // 0..1

          if (left != 0.0) {
            launcher.setPercent(left);

          } else {
            launcher.stop();
          }
        },
        launcher);
  }
}
