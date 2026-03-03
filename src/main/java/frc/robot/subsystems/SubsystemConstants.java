/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * This class contains constants used to control and configure all the <code>Subsystems</code> that
 * make up the robot.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.stu
 * @version 2026.0.0
 */
public final class SubsystemConstants {

  public static final boolean doSysId = false;

  /*
   * Subsystem Overall Control Constants
   *
   * <p> Each subsystem gets a boolean to control whether to use it (create and enable) as well as a
   * name (used for ensuring that telemetry all ends up in the same location in the tree and
   * prevents typos - these are used to auto-populate the dashboard tabs).
   */
  public static final boolean useVision = false;

  public static final String visionName = "Vision";

  public static final boolean useShooter = false;
  public static final String shooterName = "Shooter";
  public static final boolean useTurret = false;
  public static final String turretName = "Turret";

  public static final boolean useHopper = false;
  public static final String hopperName = "Hopper";
  public static final boolean useIntake = false;
  public static final String intakeName = "Intake";
  public static final boolean useLift = false;
  public static final String liftName = "Lift";

  public static final boolean useClimber = false;
  public static final String climberName = "Climber";

  public static final boolean useLauncher = false;
  public static final String launcherName = "Launcher";

  /*
   * Standard telemetry names.
   */
  public static final String tlmStatusName = "/Status";
  public static final String tlmRevLibErrorName = "/isRevLibError";
  // TODO: Is there an equivalent CTRE library error to capture?
}
