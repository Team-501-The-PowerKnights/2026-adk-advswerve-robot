package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;

  public VisionIOLimelight(String limelightTableName) {
    table = NetworkTableInstance.getDefault().getTable(limelightTableName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.hasTarget = table.getEntry("tv").getDouble(0.0) > 0.5;

    inputs.botPoseWpiBlue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

    // tl and cl are in ms
    double tlMs = table.getEntry("tl").getDouble(0.0);
    double clMs = table.getEntry("cl").getDouble(0.0);
    inputs.pipelineLatencySec = tlMs / 1000.0;
    inputs.captureLatencySec = clMs / 1000.0;

    // botpose arrays include latency + tag count after pose:
    // [0..2]=XYZ(m), [3..5]=RPY(deg), [6]=latency(ms), [7]=tag count
    if (inputs.botPoseWpiBlue.length >= 8) {
      inputs.tagCount = (int) inputs.botPoseWpiBlue[7];
      double latencyMs = inputs.botPoseWpiBlue[6];
      inputs.timestampSec = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    } else {
      inputs.tagCount = 0;
      // Fall back to tl+cl timestamp if botpose array is short
      inputs.timestampSec =
          Timer.getFPGATimestamp() - (inputs.pipelineLatencySec + inputs.captureLatencySec);
    }
  }

  @Override
  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public void setLedMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }
}
