// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.lift.Lift;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftCommands extends Command {

  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  /** Private constructor so can't be instantiated externally */
  private LiftCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * @param lift
   * @return
   */
  public static Command stop(Lift lift) {
    return lift.runOnce(lift::stop).withName("LiftStop");
  }

  /**
   * @param lift
   * @param task
   * @return
   */
  public static Command setTask(Lift lift, Lift.Task task) {
    return Commands.runOnce(
        () -> {
          lift.setTask(task);
        },
        lift);
  }

  /**
   * Command to do a manual control of the subystem for debugging.
   *
   * <p>Every subsystem <b>has</b> to have this to allow debug.
   *
   * @param lift
   * @param speedSupplier
   * @return
   */
  public static Command debugManual(Lift lift, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          lift.acceptInput(speed);
        },
        lift);
  }
}
