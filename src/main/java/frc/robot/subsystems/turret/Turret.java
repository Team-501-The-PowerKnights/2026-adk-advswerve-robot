package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final SparkMax motor = new SparkMax(TurretConstants.kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private double targetAngleDeg = 0.0;
  private boolean autoAimEnabled = false;
  private final PIDController anglePid =
      new PIDController(
          TurretConstants.kAutoAimKp, TurretConstants.kAutoAimKi, TurretConstants.kAutoAimKd);

  public Turret() {
    // Build config (this is where idle mode + inversion live in this API generation)
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(TurretConstants.kMotorInverted);
    cfg.idleMode(TurretConstants.kIdleMode);

    // Encoder reports turret degrees:
    double posFactorDegPerMotorRot = 360.0 / TurretConstants.kMotorRotationsPerTurretRotation;
    cfg.encoder.positionConversionFactor(posFactorDegPerMotorRot);
    cfg.encoder.velocityConversionFactor(posFactorDegPerMotorRot / 60.0); // deg/sec

    // Use the NON-deprecated overload: com.revrobotics.ResetMode / PersistMode
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Rear-facing at boot = 0 degrees
    encoder.setPosition(0.0);
    anglePid.setTolerance(TurretConstants.kAutoAimDeadbandDeg);
    anglePid.enableContinuousInput(-180.0, 180.0); // angles wrap nicely
  }

  public void turnLeft() {
    if (getAngleDeg() > -TurretConstants.kMaxAngleDeg) {
      motor.set(-TurretConstants.kManualDutyCycle);
    } else {
      motor.set(0.0);
    }
  }

  public void turnRight() {
    if (getAngleDeg() < TurretConstants.kMaxAngleDeg) {
      motor.set(TurretConstants.kManualDutyCycle);
    } else {
      motor.set(0.0);
    }
  }

  public void stop() {
    motor.set(0.0);
  }

  public double getAngleDeg() {
    return encoder.getPosition(); // degrees
  }
  /** Set a turret angle setpoint in turret coordinates (0 = rear, + = left, - = right). */
  public void setTargetAngleDeg(double turretAngleDeg) {
    targetAngleDeg =
        MathUtil.clamp(turretAngleDeg, -TurretConstants.kMaxAngleDeg, TurretConstants.kMaxAngleDeg);
  }

  public void enableAutoAim(boolean enable) {
    autoAimEnabled = enable;
    if (enable) anglePid.reset();
    else stop();
  }

  public boolean atTarget() {
    return anglePid.atSetpoint();
  }

  // /** Turret angle (deg), where 0 = rear at boot (your encoder convention). */
  // public double getAngleDeg() {
  //   return encoder.getPosition();
  // }

  /** Called by command loop to run the controller one step. */
  public void runAutoAim() {
    if (!autoAimEnabled) return;

    double errorBasedOutput = anglePid.calculate(getAngleDeg(), targetAngleDeg);
    double output =
        MathUtil.clamp(
            errorBasedOutput,
            -TurretConstants.kAutoAimMaxOutput,
            TurretConstants.kAutoAimMaxOutput);

    // Respect ±180° hard limits (software)
    double angle = getAngleDeg();
    if ((angle >= TurretConstants.kMaxAngleDeg && output > 0)
        || (angle <= -TurretConstants.kMaxAngleDeg && output < 0)) {
      motor.set(0.0);
      return;
    }

    motor.set(output);
  }
}
