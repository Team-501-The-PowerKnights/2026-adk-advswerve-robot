package frc.robot.subsystems.launcher;

public class LauncherConstants {
  /** CAN ID of the leader motor. All control is done through this one. */
  public static final int leaderCanId = 60;
  /** CAN ID of the follower motor. */
  public static final int followerCanId = 61;

  public static final boolean leaderInverted = true;
  /** Whether the <i>follower</i> needs to be reversed from the <i>leader</i>. */
  public static final boolean followerInverted = false;

  /** Default speed for the subsystem if nothing else is controlling it. */
  public static final double defaultSpeed = 0.9;

  public static final double defaultIdleSpeed = 0.1;

  /** */
}
