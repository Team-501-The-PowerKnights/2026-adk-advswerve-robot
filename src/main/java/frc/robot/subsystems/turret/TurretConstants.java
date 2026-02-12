/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the constants for the <code>Intake</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.Brian
 * @version 2026.0.0
 */
package frc.robot.subsystems.turret;

public class TurretConstants {

  /** CAN ID of the intake speed controller */
  static final int canId = 33;

  /** Is the motor inverted? (should be positive pulling in) */
  static final boolean turretInverted = false;
  /** */
  static final int motorCurrentLimit = 30;
  /** */
  static final double motorVoltageComp = 12.0;

  /** */
  static final double gearRatio = 5 * 5;

  // In (RightTraverse) is +, out (LeftTraverse) is -
  static final double turretRightTraverseSpeed = 0.40;
  static final double turretLeftTraverseSpeed = -0.40;
}
