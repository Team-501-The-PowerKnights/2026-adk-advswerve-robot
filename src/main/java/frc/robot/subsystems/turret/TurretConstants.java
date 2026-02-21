package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;

public final class TurretConstants {
  private TurretConstants() {}

  public static final int kMotorCanId = 32;

  // Set correctly for accurate degrees
  // Turret Motor Gear Teeth = 30
  // Turret Gear Teeth = 100
  public static final double kMotorRotationsPerTurretRotation = 100.0 / 30.0;
  public static final double kMaxAngleDeg = 180.0;
  public static final double kManualDutyCycle = 0.25;

  public static final boolean kMotorInverted = false;
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  public static final int kTurrentGearTeeth = 100;
  public static final int kTurretDriveTeeth = 30;
  // MARK: AutoAim
  // Turret center offset from Robot Center
  public static final Translation2d kTurretPivotRobotMeters =
      new Translation2d(-0.2428875, -0.2428875);
  // Field Target: Center of Hub
  // +X increases away from the Blue driver station wall
  // +Y increases to the left (when looking downfield)
  public static final Translation2d kTargetFieldMeters = new Translation2d(4.625594, 4.034536);
  // Auto-aim PID (angle control, degrees)
  public static final double kAutoAimKp = 0.020;
  public static final double kAutoAimKi = 0.000;
  public static final double kAutoAimKd = 0.001;

  public static final double kAutoAimMaxOutput = 0.35;
  public static final double kAutoAimDeadbandDeg = 1.0;

  // Turret coordinate system: 0° = facing REAR of robot
  public static final double kTurretZeroDegRobotFrameDeg = 180.0; // robot-frame rear direction
}
