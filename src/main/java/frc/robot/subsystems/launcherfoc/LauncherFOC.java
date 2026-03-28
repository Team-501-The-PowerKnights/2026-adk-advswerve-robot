package frc.robot.subsystems.launcherfoc;

import static frc.robot.subsystems.launcherfoc.LauncherFOCConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.TalonFXSubsystem;
import org.littletonrobotics.junction.Logger;

public class LauncherFOC extends TalonFXSubsystem implements ISubsystem {
  private final TalonFX leader;
  private final TalonFX follower;

  /** Current target AXLE velocity in rotations/sec. */
  private double axleTargetRps = 0.0;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0.0)
          .withSlot(0)
          .withAcceleration(launcherAccelerationRpsPerSec)
          .withFeedForward(launcherFeedforwardAmps);

  private final NeutralOut neutralRequest = new NeutralOut();

  public LauncherFOC() {
    super("LauncherFOC");
    initConstruction();

    leader = new TalonFX(leaderCanId);
    follower = new TalonFX(followerCanId);

    configureMotor(leader, leaderInverted);
    configureMotor(follower, followerInverted);

    follower.setControl(
        new Follower(
            leader.getDeviceID(),
            followerOpposesLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    finishConstruction();
  }

  private void configureMotor(TalonFX motor, boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = launcherKP;
    config.Slot0.kI = launcherKI;
    config.Slot0.kD = launcherKD;
    config.Slot0.kS = launcherKS;
    config.Slot0.kV = launcherKV;
    config.Slot0.kA = launcherKA;

    config.TorqueCurrent.PeakForwardTorqueCurrent = peakForwardTorqueCurrentAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = peakReverseTorqueCurrentAmps;
    config.TorqueCurrent.TorqueNeutralDeadband = torqueNeutralDeadbandAmps;

    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = enableStatorCurrentLimit;

    StatusCode status = motor.getConfigurator().apply(config);
    if (!status.isOK()) {
      System.out.println(
          "Failed to configure TalonFX " + motor.getDeviceID() + ": " + status.toString());
    }
  }

  public void stop() {
    axleTargetRps = 0.0;
    leader.setControl(neutralRequest);
  }

  public void acceptInput(double input) {
    double clamped = clamp(input, -1.0, 1.0);
    axleTargetRps = clamped * maxManualRps;
  }

  /** Set target in AXLE rotations/sec. */
  public void setTargetRps(double rps) {
    axleTargetRps = rps;
  }

  public void setNear() {
    axleTargetRps = kLauncherNearRps;
  }

  public void setMid() {
    axleTargetRps = kLauncherMidRps;
  }

  public void setFar() {
    axleTargetRps = kLauncherFarRps;
  }

  public void pullInNear() {
    axleTargetRps = kLauncherNearRps;
  }

  public void pullInMid() {
    axleTargetRps = kLauncherMidRps;
  }

  public void pullInFar() {
    axleTargetRps = kLauncherFarRps;
  }

  public void launcherIdle() {
    axleTargetRps = defaultIdleRps;
  }

  public void setIdle() {
    // System.out.println("****** isInPit " + RobotContainer.isInPit());
    axleTargetRps = RobotContainer.isInPit() ? 0.0 : defaultIdleRps;
  }

  private double axleRpsToMotorRps(double axleRps) {
    return axleRps * launcherGearRatio;
  }

  private double motorRpsToAxleRps(double motorRps) {
    return motorRps / launcherGearRatio;
  }

  private void applyClosedLoop() {
    if (Math.abs(axleTargetRps) < launcherNeutralDeadbandRps) {
      leader.setControl(neutralRequest);
      return;
    }

    double motorTargetRps = axleRpsToMotorRps(axleTargetRps);
    leader.setControl(velocityRequest.withVelocity(motorTargetRps));
  }

  @Override
  public void disabledInit() {
    stop();
  }

  @Override
  public void periodic() {
    applyClosedLoop();
    logMotorTelemetry();
  }

  private final String tlmAxleTargetRps = getSubsystem() + "/AxleTargetRps";
  private final String tlmMotorTargetRps = getSubsystem() + "/MotorTargetRps";
  private final String tlmLeaderVelocityMotorRps = getSubsystem() + "/LeaderVelocityMotorRps";
  private final String tlmFollowerVelocityMotorRps = getSubsystem() + "/FollowerVelocityMotorRps";
  private final String tlmLeaderVelocityAxleRps = getSubsystem() + "/LeaderVelocityAxleRps";
  private final String tlmFollowerVelocityAxleRps = getSubsystem() + "/FollowerVelocityAxleRps";
  private final String tlmLeaderTorqueCurrent = getSubsystem() + "/LeaderTorqueCurrentAmps";
  private final String tlmFollowerTorqueCurrent = getSubsystem() + "/FollowerTorqueCurrentAmps";

  private void logMotorTelemetry() {
    double leaderMotorRps = leader.getVelocity().getValueAsDouble();
    double followerMotorRps = follower.getVelocity().getValueAsDouble();

    Logger.recordOutput(tlmAxleTargetRps, axleTargetRps);
    Logger.recordOutput(tlmMotorTargetRps, axleRpsToMotorRps(axleTargetRps));
    Logger.recordOutput(tlmLeaderVelocityMotorRps, leaderMotorRps);
    Logger.recordOutput(tlmFollowerVelocityMotorRps, followerMotorRps);
    Logger.recordOutput(tlmLeaderVelocityAxleRps, motorRpsToAxleRps(leaderMotorRps));
    Logger.recordOutput(tlmFollowerVelocityAxleRps, motorRpsToAxleRps(followerMotorRps));
    Logger.recordOutput(tlmLeaderTorqueCurrent, leader.getTorqueCurrent().getValueAsDouble());
    Logger.recordOutput(tlmFollowerTorqueCurrent, follower.getTorqueCurrent().getValueAsDouble());

    Logger.recordOutput(
        getSubsystem() + "/isLeaderNearTarget", leader.getVelocity().isNear(axleTargetRps, 0.5));
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
