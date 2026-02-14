package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final SparkMax leader = new SparkMax(HopperConstants.kLeaderCanId, MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(HopperConstants.kFollowerCanId, MotorType.kBrushless);

  public Hopper() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.follow(leader, HopperConstants.kFollowerInverted);

    leader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Runs hopper at a percent output (-1 to 1). */
  public void setPercent(double percent) {
    leader.set(percent);
  }

  public void stop() {
    leader.stopMotor();
  }
}
