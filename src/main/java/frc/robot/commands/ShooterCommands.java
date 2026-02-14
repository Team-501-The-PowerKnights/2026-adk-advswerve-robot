package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public final class ShooterCommands {
  private ShooterCommands() {}

  /**
   * Drives shooter from two analog triggers: - leftTrigger: shoots out (positive) - rightTrigger:
   * reverses (negative)
   *
   * <p>If both are pressed, whichever is larger wins.
   */
  public static Command triggerDrive(
      Shooter shooter, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    return new RunCommand(
        () -> {
          double left = leftTrigger.getAsDouble(); // 0..1
          double right = rightTrigger.getAsDouble(); // 0..1

          if (left > right) {
            shooter.setPercent(left);
          } else if (right > left) {
            shooter.setPercent(-right);
          } else {
            shooter.stop();
          }
        },
        shooter);
  }
}
