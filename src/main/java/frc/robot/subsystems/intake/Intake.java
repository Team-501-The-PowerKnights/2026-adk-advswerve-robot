package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkFlex motor = new SparkFlex(IntakeConstants.kMotorCanId, MotorType.kBrushless);

  public Intake() {
    SparkFlexConfig cfg = new SparkFlexConfig();
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
