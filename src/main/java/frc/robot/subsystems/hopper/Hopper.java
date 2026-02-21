package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final SparkFlex leader =
      new SparkFlex(HopperConstants.kLeaderCanId, MotorType.kBrushless);
  private final SparkFlex follower =
      new SparkFlex(HopperConstants.kFollowerCanId, MotorType.kBrushless);

  public Hopper() {
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig.idleMode(IdleMode.kCoast);

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.idleMode(IdleMode.kCoast);
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
