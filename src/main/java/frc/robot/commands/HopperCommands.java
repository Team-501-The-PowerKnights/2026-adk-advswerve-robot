// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import java.util.function.DoubleSupplier;

public class HopperCommands {
  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  /** Private constructor so can't be instantiated externally */
  private HopperCommands() {}

  public static Command manual(Hopper hopper, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          hopper.acceptTeleopInput(speed);
        },
        hopper);
  }
}
