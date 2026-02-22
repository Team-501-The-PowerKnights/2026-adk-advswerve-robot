/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/
package frc.robot.subsystems;

import static frc.robot.subsystems.SubsystemConstants.*;
import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * This class contains the implementation of a common class for RevRobotics-based subsystems.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.stu
 * @version 2026.0.0
 */
public abstract class RevRoboticsSubsystem extends SubsystemBase {

  /** Name of the subsystem (saved from constructor) */
  private final String name;

  /** Original value of sticky fault (so it can be restored at the end) */
  private boolean origSparkStickyFault;

  /** Constructs a new instance of the subsystem. */
  public RevRoboticsSubsystem(String subsystemName) {
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
    origSparkStickyFault = sparkStickyFault;
  }

  /**
   * Finishes the construction of the <code>Subsystem</code>. Should be called once at the very end
   * of the constructor.
   */
  @SuppressWarnings("resource")
  protected void finishConstruction() {
    // Log this subsystem's status and return global
    Logger.recordOutput(name + tlmRevLibErrorName, !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in " + name + " construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib " + name + " construction", AlertType.kInfo).set(true);
    }
    boolean mySparkStickyFault = sparkStickyFault;
    sparkStickyFault |= origSparkStickyFault;

    boolean constructStatus = true && mySparkStickyFault;
    Logger.recordOutput(name + tlmStatusName, constructStatus);
  }
}
