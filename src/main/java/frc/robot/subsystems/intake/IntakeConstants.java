/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

/**
 * This class contains the constants for the <code>Intake</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.brian Buzzell
 * @version 2026.0.0
 */
public final class IntakeConstants {

  /** CAN ID of the motor. */
  public static final int motorCanId = 40;

  /** Whether the <i>motor</i> needs to be reversed */
  public static final boolean motorInverted = true;

  /** Default speed for the subsystem if nothing else is controlling it. */
  public static final double defaultSpeed = 0.85;

  /** */
  // TODO - Set real value for motorCurrentLimit
  static final int motorCurrentLimit = 25;
  /** */
  // TODO - Set real value for motorVoltageComp
  static final double motorVoltageComp = 12.0;

  /** Trigger value for motor overtemp (C) */
  static final double motorOverTemp = 60.0; // seems to be < 50 in the pits
}
