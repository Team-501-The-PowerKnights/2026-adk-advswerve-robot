// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.SubsystemConstants.launcherName;
import static frc.robot.subsystems.launcher.LauncherConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.TalonFXSubsystem;
import org.littletonrobotics.junction.Logger;

public class Launcher extends TalonFXSubsystem implements ISubsystem {
  private final TalonFX leader; // = new TalonFX(leaderCanId);
  private final TalonFX follower; // = new TalonFX(followerCanId);
  private double currentSpeed = 0.0;

  /** Creates a new Launcher. */
  public Launcher() {
    super(launcherName);
    initConstruction();

    leader = new TalonFX(leaderCanId);
    follower = new TalonFX(followerCanId);

    // Follow leader, inverted relative to leader
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    finishConstruction();
  }

  public void stop() {
    acceptInput(0.0);
    leader.stopMotor();
  }

  /** Percent output: -1.0 to +1.0 */
  public void acceptInput(double speed) {
    currentSpeed = speed;
  }
  /**
   * Sets the desired speed on the motor.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  private void setSpeed(double speed) {
    leader.set(speed);
    // TODO - Figure out how to use DutyCycleOut with Follower control
    // leader.setControl(duty.withOutput(speed));
  }

  public void launcherIdle() {
    currentSpeed = LauncherConstants.defaultIdleSpeed;
  }

  public void setIdle() {
    currentSpeed = defaultIdleSpeed;
    leader.set(defaultIdleSpeed);
  }

  @Override
  public void disabledInit() {
    // Ensure any motion stops when we go to <i>disabled</code>
    stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setSpeed(currentSpeed);
    logMotorTelemetry(leader, follower);
  }

  private final String tlmLeaderSpeed = getSubsystem() + "/LeaderSpeed";
  private final String tlmFollowerSpeed = getSubsystem() + "/FollowerSpeed";
  // private final String tlmLeaderCurrent = getSubsystem() + "/LeaderCurrent";
  // private final String tlmLeaderTemp = getSubsystem() + "/LeaderTemp";
  private final String tlmCurrentSpeed = getSubsystem() + "/CurrentSpeed";

  /**
   * Log the telemetry so it can be recorded and placed on the dashboard. This version is for a
   * <code>Subsystem</code> that has a <i>motor</i> and a <i>follower</i>.
   *
   * @param leader
   * @param follower
   */
  private void logMotorTelemetry(TalonFX leader, TalonFX follower) {
    // TODO - Add telemetry for motors
    Logger.recordOutput(tlmCurrentSpeed, currentSpeed);
    Logger.recordOutput(tlmLeaderSpeed, leader.get());
    Logger.recordOutput(tlmFollowerSpeed, follower.get());
  }
}
