package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lift.Lift.Position;

public final class SuperstructureCommands {
  private SuperstructureCommands() {}

  /** A button toggle: UP->DOWN starts intake, DOWN->UP stops intake. */
  public static Command toggleLiftAndIntake(Lift lift, Intake intake) {
    return Commands.runOnce(
            () -> {
              // Decide based on current GOAL (more deterministic than position mid-move)
              if (lift.getGoal() == Position.UP) {
                lift.setGoal(Position.DOWN);
                intake.setPercent(IntakeConstants.kIntakeSpeed);
              } else {
                lift.setGoal(Position.UP);
                intake.stop();
              }
            },
            lift,
            intake)
        .withName("ToggleLiftAndIntake");
  }

  /** Optional: enforce startup state. */
  public static Command forceUpAndStop(Lift lift, Intake intake) {
    return Commands.runOnce(
            () -> {
              lift.setGoal(Position.UP);
              intake.stop();
            },
            lift,
            intake)
        .withName("ForceUpAndStop");
  }
}
