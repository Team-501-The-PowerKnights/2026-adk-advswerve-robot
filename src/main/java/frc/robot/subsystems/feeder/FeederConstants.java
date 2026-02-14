package frc.robot.subsystems.feeder;

public final class FeederConstants {
  private FeederConstants() {}

  public static final int kMotorCanId = 52;

  public static final double kFeedForwardSpeed = 0.60;
  public static final double kFeedReverseSpeed = -0.60;

  public static final int kCurrentLimitAmps = 40; // ✅ int (not double)
  public static final boolean kInvertMotor = false;
}
