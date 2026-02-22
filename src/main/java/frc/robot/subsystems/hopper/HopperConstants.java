package frc.robot.subsystems.hopper;

public final class HopperConstants {
  private HopperConstants() {}

  public static final int kLeaderCanId = 50;
  public static final int kFollowerCanId = 51;

  /** Leave false until you test. If motors fight, flip to true. */
  public static final boolean kFollowerInverted = false;

  /** Percent output (0 to 1). */
  public static final double kSpeed = 0.75;
}
