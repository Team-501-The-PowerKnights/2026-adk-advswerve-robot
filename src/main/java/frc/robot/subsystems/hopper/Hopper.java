/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.SubsystemConstants.hopperName;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.RevRoboticsSubsystem;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

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

  /** */
  private final SparkFlex leader;
  /** */
  private final SparkFlex follower;

  /** */
  private double currentSpeed;

  /** Constructs a new instance of the subsystem. */
  public Hopper() {
    super(hopperName);
    initConstruction();

    // Create and configure leader
    leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig.idleMode(IdleMode.kCoast);
    SparkUtil501.tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create and configure follower
    follower = new SparkFlex(followerCanId, MotorType.kBrushless);
    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.idleMode(IdleMode.kCoast).follow(leader, followerInverted);
    SparkUtil501.tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // We start stopped
    currentSpeed = 0;
    setSpeed(currentSpeed);

    finishConstruction();
  }

  // TODO - Document directions for +/- speed
  /**
   * Accepts a manual override of any defaults or PID controlled set points to allow <i>Operator</i>
   * adjustment of the position. Positive values ???? and negative values ????.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  public void acceptTeleopInput(double speed) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }
    currentSpeed = speed;
  }

  /**
   * Sets the desired speed on the motor.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  private void setSpeed(double speed) {
    leader.set(speed);
  }

  @Override
  public void disabledInit() {
    // Ensure any motion stops when we go to <i>disabled</code>
    setSpeed(0);
    // TODO - Remember why we don't call motor.stop() vs. setting speed to 0 (or else fix)
  }

  // TODO - Add code to set the default speed on auto & teleop if running that way

  @Override
  public void periodic() {
    setSpeed(currentSpeed);

    Logger.recordOutput(hopperName + "/CurrentSpeed", currentSpeed);
    Logger.recordOutput(hopperName + "/Output", leader.get());
  }
}
