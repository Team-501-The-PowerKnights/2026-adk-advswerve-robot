// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.ISubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  // Hardware objects
  private final SparkMax motor;

  private double currentSpeed;
  /** Creates a new Turret. */
  public Turret() {
    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;

    // Create controller
    motor = new SparkMax(TurretConstants.canId, MotorType.kBrushless);
    // Factory reset (but don't burn to flash)
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(TurretConstants.turretInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.motorCurrentLimit)
        .voltageCompensation(TurretConstants.motorVoltageComp);
    SparkUtil501.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Log this subsystem's status and return global
    Logger.recordOutput("Turret/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in Turret construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib Turret construction", AlertType.kInfo).set(true);
    }
    SparkUtil501.sparkStickyFault |= origSparkStickyFault;
  }

  /**
   * Accepts a manual override of the PID controlled set points to allow <i>Operator</i> adjustment
   * of the position. Positive values lift and negative values lower.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  public void acceptTeleopInput(double speed) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }
    currentSpeed = speed;
  }

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public void periodic() {
    setSpeed(currentSpeed);

    Logger.recordOutput("Turret/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Turret/Output", motor.get());
  }
}
