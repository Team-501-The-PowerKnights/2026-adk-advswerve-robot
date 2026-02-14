package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode; // ✅ note package
import com.revrobotics.ResetMode; // ✅ note package
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final SparkMax motor;

  public Feeder() {
    motor = new SparkMax(FeederConstants.kMotorCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(FeederConstants.kInvertMotor)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.kCurrentLimitAmps);

    // ✅ Use the non-deprecated overload (com.revrobotics.ResetMode/PersistMode)
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stop();
  }

  public void set(double percent) {
    motor.set(percent);
  }

  public void stop() {
    motor.set(0.0);
  }
}
