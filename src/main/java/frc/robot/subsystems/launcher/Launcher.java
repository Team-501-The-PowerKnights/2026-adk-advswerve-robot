// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.SubsystemConstants.launcherName;
import static frc.robot.subsystems.launcher.LauncherConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.TalonFXSubsystem;

public class Launcher extends TalonFXSubsystem implements ISubsystem {
  private final TalonFX leader = new TalonFX(leaderCanId);
  private final TalonFX follower = new TalonFX(followerCanId);

  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);
  /** Creates a new Launcher. */
  public Launcher() {
    super(launcherName);
    initConstruction();
    // Follow leader, inverted relative to leader
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }
  /** Percent output: -1.0 to +1.0 */
  public void setPercent(double percent) {
    leader.setControl(duty.withOutput(percent));
  }

  public void stop() {
    setPercent(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
