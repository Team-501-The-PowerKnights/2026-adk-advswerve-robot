package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lift.Lift.Position;

public final class LiftCommands {
  private LiftCommands() {}

  public static Command moveUp(Lift lift) {
    return Commands.runOnce(() -> lift.setGoal(Position.UP), lift).withName("LiftUp");
  }

  public static Command moveDown(Lift lift) {
    return Commands.runOnce(() -> lift.setGoal(Position.DOWN), lift).withName("LiftDown");
  }

  public static Command toggle(Lift lift) {
    return Commands.runOnce(lift::toggleGoal, lift).withName("LiftToggle");
  }
}
