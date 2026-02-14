package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;

public class FeederCommands {
  private final Feeder feeder;

  public FeederCommands(Feeder feeder) {
    this.feeder = feeder;
  }

  public Command feedForward() {
    return feeder
        .run(() -> feeder.set(FeederConstants.kFeedForwardSpeed))
        .withName("FeederForward");
  }

  public Command feedReverse() {
    return feeder
        .run(() -> feeder.set(FeederConstants.kFeedReverseSpeed))
        .withName("FeederReverse");
  }

  public Command stop() {
    return feeder.runOnce(feeder::stop).withName("FeederStop");
  }
}
