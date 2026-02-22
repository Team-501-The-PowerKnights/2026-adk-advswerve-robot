/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.SubsystemConstants.hopperName;

import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.RevRoboticsSubsystem;

/**
 * This class contains the implementation of the <code>Hopper</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.brian Buzzell
 * @version 2026.0.0
 */
public class Hopper extends RevRoboticsSubsystem implements ISubsystem {

  /** Constructs a new instance of the subsystem. */
  public Hopper() {
    super(hopperName);
    initConstruction();

    finishConstruction();
  }

  @Override
  public void periodic() {}
}
