package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.lift.Lift;

public final class IntakeCommands {
  private IntakeCommands() {}

  /**
   * Runs intake while this command is scheduled, BUT only if the lift is down. If lift is not down,
   * intake is forced off.
   */
  public static Command runWhenLiftDown(Intake intake, Lift lift) {
    return Commands.run(
            () -> {
              if (lift.isDown()) {
                intake.setPercent(IntakeConstants.kIntakeSpeed);
              } else {
                intake.stop();
              }
            },
            intake)
        .finallyDo(intake::stop)
        .withName("IntakeRunWhenLiftDown");
  }

  public static Command reverseWhenLiftDown(Intake intake, Lift lift) {
    return Commands.run(
            () -> {
              if (lift.isDown()) {
                intake.setPercent(IntakeConstants.kOuttakeSpeed);
              } else {
                intake.stop();
              }
            },
            intake)
        .finallyDo(intake::stop)
        .withName("IntakeReverseWhenLiftDown");
  }

  /** Default behavior: run only when lift is down */
  public static Command autoRun(Intake intake, Lift lift) {
    return Commands.run(
            () -> {
              if (lift.isDown()) {
                intake.setPercent(IntakeConstants.kIntakeSpeed);
              } else {
                intake.stop();
              }
            },
            intake)
        .withName("IntakeAutoRun");
  }

  /** While held: force reverse */
  public static Command reverse(Intake intake) {
    return Commands.run(intake::reverse, intake).finallyDo(intake::stop).withName("IntakeReverse");
  }
}
