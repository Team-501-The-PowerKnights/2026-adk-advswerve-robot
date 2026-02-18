package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode; // ✅ note package
import com.revrobotics.ResetMode; // ✅ note package
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final SparkMax leader;
  private final SparkMax follower;

  public Feeder() {
    leader = new SparkMax(FeederConstants.kMotorLeaderCanId, MotorType.kBrushless);
    follower = new SparkMax(FeederConstants.kMotorFollowerCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(FeederConstants.kInvertMotor)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.kCurrentLimitAmps);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kCoast);
    followerConfig.follow(leader, FeederConstants.kInvertFollowerMotor);

    // ✅ Use the non-deprecated overload (com.revrobotics.ResetMode/PersistMode)
    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stop();
  }

  public void set(double percent) {
    leader.set(percent);
  }

  public void stop() {
    leader.set(0.0);
  }
}
