/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.lift;

/**
 * This class contains the constants for the <i>Intake</i> <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.brian Buzzell
 * @version 2026.0.0
 */
public final class LiftConstants {

  /** CAN ID of the motor. */
  public static final int motorCanId = 42; // left
  /** CAN ID of the follower motor. All control is through the leader one. */
  // public static final int followerCanId = 42; // right

  /** Is the motor inverted? (should be positive going up) */
  static final boolean motorInverted = false;

  /** Is the encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;

  /** */
  static final double gearRatio = 2; // little pulley to big pulley w/ belt

  /** Whether the <i>follower</i> needs to be reversed from the <i>leader</i>. */
  public static final boolean followerInverted = true;

  /** Default speed for the subsystem if nothing else is controlling it. */
  public static final double defaultSpeed = 0.75;

  /** */
  // TODO - Set real value for motorCurrentLimit
  static final int motorCurrentLimit = 30;
  /** */
  // TODO - Set real value for motorVoltageComp
  static final double motorVoltageComp = 12.0;

  /** Trigger value for motor overtemp (C) */
  static final double motorOverTemp = 60.0; // seems to be < 50 in the pits

  /** Whether doing PID tuning via Dashboard */
  static final boolean doPidTuning = false;

  /** PID control loop constants */
  static final double pidKp = 0.0;
  //
  static final double pidKi = 0.0;
  //
  static final double pidKd = 0.0;
  //
  static final double pidMaxPosOut = 1.0;
  //
  static final double pidMaxNegOut = -1.0;

  //
  static final double minHeight = 0.01; // Should this be higher?
  //
  static final double maxHeight = 27436.0;
}
