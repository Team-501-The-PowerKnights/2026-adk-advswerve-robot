/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/
package frc.robot.subsystems;

/**
 * This class contains the implementation of a common class for 501-based subsystems.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.stu
 * @version 2026.0.0
 */
public abstract class SubsystemBase extends edu.wpi.first.wpilibj2.command.SubsystemBase
    implements ISubsystem {

  /** */
  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  // Flag for whether first periodic() has run
  protected boolean firstPeriodic;

  // Current mode
  protected Mode currentMode;
  // If manual mode - then the current setting
  protected double currentSpeed;
  // If PID mode - then the current setting
  protected double currentTarget;

  /** */
  public SubsystemBase() {
    throw new Error("501 doesn't allow constructing Subsystems without explicit name.");
  }

  /**
   * Creates a new instance of the <code>Subsystem</code> using the provided name.
   *
   * @param name
   */
  public SubsystemBase(String name) {
    super(name);

    firstPeriodic = false;
  }
}
