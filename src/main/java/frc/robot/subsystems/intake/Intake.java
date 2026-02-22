package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax motor = new SparkMax(IntakeConstants.kMotorCanId, MotorType.kBrushless);

  public Intake() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    // Sensible defaults
    cfg.inverted(false);
    cfg.smartCurrentLimit(60);

    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Percent output [-1..1] */
  public void setPercent(double percent) {
    motor.set(percent);
  }

  public void reverse() {
    motor.set(IntakeConstants.kOuttakeSpeed);
  }

  public void stop() {
    motor.stopMotor();
  }
}
