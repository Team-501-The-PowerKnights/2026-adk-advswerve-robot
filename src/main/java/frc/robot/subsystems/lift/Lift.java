package frc.robot.subsystems.lift;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  public enum Position {
    UP,
    DOWN
  }

  private final SparkMax leader = new SparkMax(LiftConstants.kLeaderCanId, MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(LiftConstants.kFollowerCanId, MotorType.kBrushless);

  // Absolute encoder on shaft (reports a value 0..1 for one rotation)
  private final DutyCycleEncoder absEncoder =
      new DutyCycleEncoder(LiftConstants.kAbsEncoderDioPort);

  private final PIDController pid =
      new PIDController(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD);

  private Position goal = Position.UP; // starts UP

  public Lift() {
    SparkMaxConfig leaderCfg = new SparkMaxConfig();
    leaderCfg.inverted(false);
    leaderCfg.smartCurrentLimit(60);

    SparkMaxConfig followerCfg = new SparkMaxConfig();
    followerCfg.smartCurrentLimit(60);
    followerCfg.follow(leader, LiftConstants.kFollowerInverted);

    leader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // We’re not using continuous input because we do NOT want wraparound to jump
    // from 0.99 -> 0.01.
    // Our lift should live in a limited range (UP..DOWN).
    pid.setTolerance(LiftConstants.kToleranceShaftRot);

    setGoal(Position.UP);
  }

  @Override
  public void periodic() {
    double pos = getShaftPositionRot();
    double setpoint = getGoalShaftRot(goal);

    double output = pid.calculate(pos, setpoint);
    output = MathUtil.clamp(output, LiftConstants.kMinOutput, LiftConstants.kMaxOutput);

    leader.set(output);
  }

  /** Shaft position in rotations, zeroed at UP via offset. */
  /** Shaft position in rotations, zeroed at UP via offset. */
  public double getShaftPositionRot() {
    // 0..1 rotations
    double rot = absEncoder.get();

    if (LiftConstants.kAbsInverted) {
      rot = 1.0 - rot;
    }

    // Apply offset so UP reads as 0
    rot -= LiftConstants.kAbsOffsetRot;

    // Wrap into [-0.5, 0.5) to avoid discontinuity surprises
    rot = rot - Math.floor(rot + 0.5);

    return rot;
  }

  public void setGoal(Position pos) {
    goal = pos;
    pid.reset(); // prevents a big derivative kick on toggles
  }

  public void toggleGoal() {
    setGoal(goal == Position.UP ? Position.DOWN : Position.UP);
  }

  public Position getGoal() {
    return goal;
  }

  public boolean isDown() {
    return Math.abs(getShaftPositionRot() - LiftConstants.kDownShaftRot)
        <= LiftConstants.kToleranceShaftRot;
  }

  public boolean isUp() {
    return Math.abs(getShaftPositionRot() - LiftConstants.kUpShaftRot)
        <= LiftConstants.kToleranceShaftRot;
  }

  private static double getGoalShaftRot(Position pos) {
    return (pos == Position.UP) ? LiftConstants.kUpShaftRot : LiftConstants.kDownShaftRot;
  }
}
