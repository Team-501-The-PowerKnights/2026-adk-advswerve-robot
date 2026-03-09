package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import java.util.function.DoubleSupplier;

public class LauncherCommands {

  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  public static Command joystickDrive(
      Launcher launcher, DoubleSupplier leftJoystick, Hopper hopper, Intake intake) {

    return new RunCommand(
        () -> {
          double left = leftJoystick.getAsDouble(); // -1..1

          if (left > DEADBAND || left < -DEADBAND) {
            launcher.acceptInput(left);
            hopper.acceptInput(-left);
          } else {
            launcher.stop();
            //Check the Intake speed, if it's not running, stop the hopper too. 
            // If the intake is running, we want the hopper to keep running to feed balls to the back of the hopper. 
            if (intake.getCurrentSpeed() == 0.0) {
              hopper.stop();
            }
          }
        },
        launcher);
  }

  public static Command joystickDrive(Launcher launcher, DoubleSupplier leftJoystick) {

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
