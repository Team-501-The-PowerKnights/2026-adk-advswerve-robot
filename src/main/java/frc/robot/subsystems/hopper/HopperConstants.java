/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.hopper;

/**
 * This class contains the constants for the <code>Hopper</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.brian Buzzell
 * @version 2026.0.0
 */
public final class HopperConstants {

  /** CAN ID of the motor. */
  public static final int motorCanId = 50;
  /** CAN ID of the follower motor. All control is through the leader one. */
  public static final int followerCanId = 51; // TODO - This is not a follower

  /** Whether the <i>follower</i> needs to be reversed from the <i>leader</i>. */
  public static final boolean motorInverted = true;
  /** Whether the <i>follower</i> needs to be reversed from the <i>leader</i>. */
  public static final boolean followerInverted = true;

  /** Default speed for the subsystem if nothing else is controlling it. */
  public static final double defaultSpeed = 0.9;

  /** */
  // TODO - Set real value for motorCurrentLimit
  static final int motorCurrentLimit = 30;
  /** */
  // TODO - Set real value for motorVoltageComp
  static final double motorVoltageComp = 12.0;

  /** Trigger value for motor overtemp (C) */
  static final double motorOverTemp = 60.0; // seems to be < 50 in the pits
}
