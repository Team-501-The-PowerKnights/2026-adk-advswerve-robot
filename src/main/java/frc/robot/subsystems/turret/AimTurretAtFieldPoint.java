package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AimTurretAtFieldPoint extends Command {
  private final Turret turret;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Translation2d> targetFieldSupplier;

  public AimTurretAtFieldPoint(
      Turret turret,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetFieldSupplier) {
    this.turret = turret;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetFieldSupplier = targetFieldSupplier;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.enableAutoAim(true);
  }

  @Override
  public void execute() {
    Pose2d robotPose = robotPoseSupplier.get();
    Translation2d targetField = targetFieldSupplier.get();

    // Turret pivot in field coordinates = robotPose + rotated turret offset
    Translation2d pivotField =
        robotPose
            .getTranslation()
            .plus(TurretConstants.kTurretPivotRobotMeters.rotateBy(robotPose.getRotation()));

    // Vector from pivot -> target in field coords
    Translation2d toTarget = targetField.minus(pivotField);

    // Desired field-facing angle to target
    Rotation2d desiredFieldAngle = new Rotation2d(toTarget.getX(), toTarget.getY());

    // Convert to robot-relative angle: desired - robotHeading
    Rotation2d desiredRobotFrame = desiredFieldAngle.minus(robotPose.getRotation());

    // Robot-frame angle is 0=forward, +90=left. Our turret 0 = rear (180 robot-frame).
    double turretSetpointDeg =
        wrapDeg(desiredRobotFrame.getDegrees() - TurretConstants.kTurretZeroDegRobotFrameDeg);

    turret.setTargetAngleDeg(turretSetpointDeg);
    turret.runAutoAim();
  }

  @Override
  public void end(boolean interrupted) {
    turret.enableAutoAim(false);
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }

  private static double wrapDeg(double deg) {
    // Wrap to [-180, 180)
    deg = deg % 360.0;
    if (deg >= 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;
    return deg;
  }
}
