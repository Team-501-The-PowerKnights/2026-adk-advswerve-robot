// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommands extends Command {

  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  /** Private constructor so can't be instantiated externally */
  private ClimberCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * @param hopper
   * @return
   */
  public static Command stop(Climber climber) {
    return null;
    // TODO: Implement Climber stop command
    // return climber.runOnce(climber::stop).withName("ClimberStop");
  }

  /**
   * Command to do a manual control of the subystem for debugging.
   *
   * <p>Every subsystem <b>has</b> to have this to allow debug.
   *
   * @param hopper
   * @param speedSupplier
   * @return
   */
  public static Command debugManual(Climber climber, DoubleSupplier speedSupplier) {
    return null;
    // TODO: Implement Climber debugManual command
    // return Commands.run(
    //     () -> {
    //       double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
    //       climber.acceptInput(speed);
    //     },
    //     climber);
  }
}
