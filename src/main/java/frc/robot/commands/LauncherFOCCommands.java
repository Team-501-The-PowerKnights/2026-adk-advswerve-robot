package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcherfoc.LauncherFOC;
import frc.robot.subsystems.launcherfoc.LauncherFOCConstants;
import java.util.function.DoubleSupplier;

public class LauncherFOCCommands {

  /** Deadband for joystick inputs. */
  private static final double DEADBAND = 0.1;

  private LauncherFOCCommands() {}

  public static Command stop(LauncherFOC launcherfoc) {
    return launcherfoc.runOnce(launcherfoc::stop).withName("LauncherFOCStop");
  }

  public static Command setIdle(LauncherFOC launcherfoc) {
    return launcherfoc.runOnce(launcherfoc::setIdle).withName("LauncherFOCSetIdle");
  }

  public static Command manual(LauncherFOC launcherfoc, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          launcherfoc.acceptInput(speed);
        },
        launcherfoc);
  }

  public static Command pushOut(LauncherFOC launcherfoc) {
    return launcherfoc
        .runEnd(
            () -> launcherfoc.acceptInput(+LauncherFOCConstants.defaultManualInput),
            launcherfoc::stop)
        .withName("LauncherFOCPushOut");
  }

  public static Command pullIn(LauncherFOC launcherfoc) {
    return launcherfoc
        .runEnd(
            () -> launcherfoc.acceptInput(-LauncherFOCConstants.defaultManualInput),
            launcherfoc::stop)
        .withName("LauncherFOCPullIn");
  }

  public static Command pullInNear(LauncherFOC launcherfoc) {
    return launcherfoc
        .runEnd(launcherfoc::pullInNear, launcherfoc::setIdle)
        .withName("LauncherFOCPullInNear");
  }

  public static Command pullInMid(LauncherFOC launcherfoc) {
    return launcherfoc
        .runEnd(launcherfoc::pullInMid, launcherfoc::setIdle)
        .withName("LauncherFOCPullInMid");
  }

  public static Command pullInFar(LauncherFOC launcherfoc) {
    return launcherfoc
        .runEnd(launcherfoc::pullInFar, launcherfoc::setIdle)
        .withName("LauncherFOCPullInFar");
  }

  public static Command setNear(LauncherFOC launcherfoc) {
    return launcherfoc.runOnce(launcherfoc::setNear).withName("LauncherFOCSetNear");
  }

  public static Command setMid(LauncherFOC launcherfoc) {
    return launcherfoc.runOnce(launcherfoc::setMid).withName("LauncherFOCSetMid");
  }

  public static Command setFar(LauncherFOC launcherfoc) {
    return launcherfoc.runOnce(launcherfoc::setFar).withName("LauncherFOCSetFar");
  }

  public static Command debugManual(LauncherFOC launcherfoc, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          launcherfoc.acceptInput(speed);
        },
        launcherfoc);
  }
}
