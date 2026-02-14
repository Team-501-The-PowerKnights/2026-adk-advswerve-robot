package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private final Matrix<N3, N1> baseStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  public Vision(VisionIO io) {
    this.io = io;

    // Conservative starting trust
    baseStdDevs.set(0, 0, 0.8); // x (m)
    baseStdDevs.set(1, 0, 0.8); // y (m)
    baseStdDevs.set(2, 0, 1.4); // yaw (rad)
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/Limelight", inputs);
  }

  public record Measurement(Pose2d pose, double timestampSec, Matrix<N3, N1> stdDevs) {}

  public Optional<Measurement> getMeasurement() {
    if (!inputs.hasTarget) return Optional.empty();
    if (inputs.botPoseWpiBlue == null || inputs.botPoseWpiBlue.length < 6) return Optional.empty();
    if (inputs.tagCount <= 0) return Optional.empty();

    double x = inputs.botPoseWpiBlue[0];
    double y = inputs.botPoseWpiBlue[1];
    double yawDeg = inputs.botPoseWpiBlue[5];

    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg));

    // More tags => more trust (down to a floor)
    double scale = 1.0 / Math.sqrt(inputs.tagCount);
    scale = Math.max(0.35, Math.min(1.0, scale));

    return Optional.of(new Measurement(pose, inputs.timestampSec, baseStdDevs.times(scale)));
  }
}
