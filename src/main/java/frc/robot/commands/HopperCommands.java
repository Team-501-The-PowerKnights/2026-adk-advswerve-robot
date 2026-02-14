package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;

public final class HopperCommands {
  private HopperCommands() {}

  public static Command forward(Hopper hopper) {
    return hopper
        .runEnd(() -> hopper.setPercent(+HopperConstants.kSpeed), hopper::stop)
        .withName("HopperForward");
  }

  public static Command reverse(Hopper hopper) {
    return hopper
        .runEnd(() -> hopper.setPercent(-HopperConstants.kSpeed), hopper::stop)
        .withName("HopperReverse");
  }
}
