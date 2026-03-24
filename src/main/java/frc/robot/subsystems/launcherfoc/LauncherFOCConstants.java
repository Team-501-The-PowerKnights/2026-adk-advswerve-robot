package frc.robot.subsystems.launcherfoc;

public class LauncherFOCConstants {
  /** CAN ID of the leader motor. */
  public static final int leaderCanId = 60;

  /** CAN ID of the follower motor. */
  public static final int followerCanId = 61;

  /** Motor inversion settings. */
  public static final boolean leaderInverted = true;

  public static final boolean followerInverted = false;

  /**
   * Set true if the follower must oppose the leader because the motors are mirrored mechanically.
   */
  public static final boolean followerOpposesLeader = true;

  /** Motor rotations per axle rotation. 2:1 means motor spins twice for 1 axle rotation. */
  public static final double launcherGearRatio = 2.0;

  /** Manual/default normalized input used by commands. */
  public static final double defaultManualInput = 0.30;

  /** Idle target in AXLE rotations/sec. */
  public static final double defaultIdleRps = 10.0;

  /** Explicit launcher presets in AXLE rotations/sec. Tune these on the real robot. */
  public static final double kLauncherNearRps = 12.0;

  public static final double kLauncherMidRps = 14.0;
  public static final double kLauncherFarRps = 16.0;

  /**
   * Maximum manual target in AXLE rotations/sec. Full input maps to this axle speed, then gets
   * converted to motor speed internally.
   */
  public static final double maxManualRps = 90.0;

  /** Neutral deadband for target axle velocity. */
  public static final double launcherNeutralDeadbandRps = 1.0;

  /** Requested motor acceleration for velocity closed-loop. */
  public static final double launcherAccelerationRpsPerSec = 300.0;

  /** Additional torque-current feedforward in amps. Start at 0 and tune only if needed. */
  public static final double launcherFeedforwardAmps = 0.0;

  /** Slot 0 gains for VelocityTorqueCurrentFOC. */
  public static final double launcherKP = 10.0; // Was .18

  public static final double launcherKV = 0.6;
  public static final double launcherKI = 0.0;
  public static final double launcherKD = 0.01;
  public static final double launcherKS = 10.0;
  public static final double launcherKA = 0.0;

  /** Torque-current limits for Phoenix Pro FOC. */
  public static final double peakForwardTorqueCurrentAmps = 80.0;

  public static final double peakReverseTorqueCurrentAmps = -80.0;
  public static final double torqueNeutralDeadbandAmps = 0.0;

  /** Optional stator current limiting. */
  public static final boolean enableStatorCurrentLimit = true;

  public static final double statorCurrentLimitAmps = 100.0;

  private LauncherFOCConstants() {}
}
