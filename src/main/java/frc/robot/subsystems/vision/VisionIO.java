package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean hasTarget = false;

    /** botpose_wpiblue: [x,y,z,roll,pitch,yaw,latencyMs,tagCount,...] */
    public double[] botPoseWpiBlue = new double[0];

    public double timestampSec = 0.0;
    public double pipelineLatencySec = 0.0;
    public double captureLatencySec = 0.0;

    public int tagCount = 0;
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setPipeline(int pipeline) {}

  default void setLedMode(int mode) {}
}
