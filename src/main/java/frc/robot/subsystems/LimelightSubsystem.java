/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/
package frc.robot.subsystems;

import static frc.robot.subsystems.SubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class LimelightSubsystem extends SubsystemBase {
  /** Name of the subsystem (saved from constructor) */
  private final String name;
  
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(String subsystemName) { 
    super(subsystemName);
    name = subsystemName;

  }
  /**
   * Initializes the construction of the <code>Subsystem</code>. Should be called once at the very
   * start of the constructor.
   */
  protected void initConstruction() {
    // Initialize status for capturing class construction
    Logger.recordOutput(name + tlmStatusName, false); // red=not OK
  }

  /**
   * Finishes the construction of the <code>Subsystem</code>. Should be called once at the very end
   * of the constructor.
   */
  @SuppressWarnings("resource")
  protected void finishConstruction() {
    // Log this subsystem's status and return global

    boolean constructStatus = true;
    Logger.recordOutput(name + tlmStatusName, constructStatus);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
