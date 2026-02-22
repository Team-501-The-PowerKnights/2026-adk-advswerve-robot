package frc.robot.subsystems.lift;

public final class LiftConstants {
  private LiftConstants() {}

  public static final int kLeaderCanId = 41;
  public static final int kFollowerCanId = 42;

  // Absolute encoder is on the shaft (50T gear shaft)
  public static final int kAbsEncoderDioPort = 0; // CHANGE to your wiring

  // Gear ratio: 25T motor gear drives 50T shaft gear => motor:shaft = 50/25 = 2.0
  // (motor spins 2 rotations per 1 shaft rotation)
  public static final double kMotorRotPerShaftRot = 2.0;

  // Absolute encoder configuration
  // Offset so that when the lift is physically at "UP", the measured position becomes 0 rotations.
  // Measure the raw absolute reading (0..1) at UP, then set this to that value.
  public static final double kAbsOffsetRot = 0.0; // CALIBRATE
  public static final boolean kAbsInverted = false;

  // Lift positions in SHAFT rotations (because encoder is on shaft)
  public static final double kUpShaftRot = 0.0;
  public static final double kDownShaftRot = 1.25; // CALIBRATE

  public static final double kToleranceShaftRot = 0.02;

  // PID driving percent output
  public static final double kP = 0.0; // CALIBRATE
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kMaxOutput = 1.0;
  public static final double kMinOutput = -1.0;

  public static final boolean kFollowerInverted = true;
}
