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

  /** CAN ID of the leader motor. All control is done through this one. */
  public static final int leaderCanId = 50;
  /** CAN ID of the follower motor. */
  public static final int followerCanId = 51;

  /** Whether the <i>follower</i> needs to be reversed from the <i>leader</i>. */
  public static final boolean followerInverted = false;

  /** Default speed for the subsystem if nothing else is controlling it. */
  public static final double defaultSpeed = 0.75;
}
