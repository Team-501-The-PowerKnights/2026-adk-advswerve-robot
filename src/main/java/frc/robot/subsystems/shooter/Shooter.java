package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX leader = new TalonFX(LeaderCanID);
  private final TalonFX follower = new TalonFX(FollowerCanID);

  private final DutyCycleOut duty = new DutyCycleOut(0).withEnableFOC(true);

  public Shooter() {
    // Follow leader, inverted relative to leader
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /** Percent output: -1.0 to +1.0 */
  public void setPercent(double percent) {
    leader.setControl(duty.withOutput(percent));
  }

  public void stop() {
    setPercent(0.0);
  }
}
