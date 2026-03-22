// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcherfoc.LauncherFOC;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommands extends Command {
  private static final double FIRST_DRIVE_FORWARD_SPEED_MPS = 1.50;
  private static final double FIRST_DRIVE_FORWARD_TIME_SEC = 1.2;

  private static final double SHOOT_SPINUP_SEC = 4.00;
  private static final double SHOOT_FEED_SEC = 5.00;

  private static final double COLLECT_FORWARD_SPEED_MPS = -1.5;
  private static final double COLLECT_FORWARD_TIME_SEC = 3.00;

  private static final double STRAFE_RIGHT_SPEED_MPS = 1.50;
  private static final double STRAFE_RIGHT_TIME_SEC = 1.4;

  private static final double RETURN_SPEED_MPS = 1.75;
  private static final double RETURN_TIME_SEC = 3.00;

  private AutoCommands() {}

  /**
   * Main Auto V1: 1) Drive forward off the start 2) Shoot preloaded fuel 3) Drive to midfield while
   * collecting 4) Return toward hub 5) Shoot collected fuel
   */
  public static Command redCenterHubAutoV1(
      Drive drive, LauncherFOC launcherfoc, Hopper hopper, Intake intake) {
    return Commands.sequence(
        stopDrive(drive),
//This moves the robot back 75 inches.
        // 1) Drive forward from the starting line
        driveRobotRelative(drive, FIRST_DRIVE_FORWARD_SPEED_MPS, FIRST_DRIVE_FORWARD_TIME_SEC),
        stopDrive(drive),

        // 2) Shoot the 8 preloaded fuel
        shootFuel(launcherfoc, hopper, null),

        // // 3) Drive toward midfield and collect
        // Commands.deadline(
        //     Commands.waitSeconds(COLLECT_FORWARD_TIME_SEC),
        //     driveRobotRelativeContinuous(drive, COLLECT_FORWARD_SPEED_MPS),
        //     IntakeCommands.pullIn(intake),
        //     HopperCommands.pullIn(hopper)),

        // stopDrive(drive),

        // // 4) Drive back toward the hub area
        // driveRobotRelative(drive, RETURN_SPEED_MPS, RETURN_TIME_SEC),

        // stopDrive(drive),

        // // 5) Shoot whatever was collected
        // shootFuel(launcher, hopper, intake),

        stopDrive(drive));
  }

  public static Command redCenterHubAutoV2(
      Drive drive, LauncherFOC launcherfoc, Hopper hopper, Intake intake) {
    return Commands.sequence(
        stopDrive(drive),

        // 1) Drive forward from the starting line
        driveRobotRelative(drive, FIRST_DRIVE_FORWARD_SPEED_MPS, FIRST_DRIVE_FORWARD_TIME_SEC),
        stopDrive(drive),

        // 2) Shoot the 8 preloaded fuel
        shootFuel(launcherfoc, hopper, null),

        // 3 strafe to the right:
        strafeRobotRelative(drive, STRAFE_RIGHT_TIME_SEC, STRAFE_RIGHT_SPEED_MPS),
        stopDrive(drive),
        driveRobotRelative(drive, COLLECT_FORWARD_SPEED_MPS, COLLECT_FORWARD_TIME_SEC),
        stopDrive(drive));
  }
  /** Spins up launcher, then feeds fuel. */
  // private static Command shootFuel( Launcher launcher, Hopper hopper, Intake intake, IntakeLift
  // intakelift) {
  //   if (intake == null) {
  //     return Commands.deadline(
  //         Commands.waitSeconds(SHOOT_SPINUP_SEC + SHOOT_FEED_SEC),
  //         LauncherCommands.pullInMid(launcher),
  //         Commands.sequence(Commands.waitSeconds(SHOOT_SPINUP_SEC),
  // HopperCommands.pullIn(hopper)),
  //         IntakeLiftCommands.raise(intakelift).withTimeout(1.0));
  //   }

  //   return Commands.deadline(
  //       Commands.waitSeconds(SHOOT_SPINUP_SEC + SHOOT_FEED_SEC),
  //       LauncherCommands.pullInMid(launcher),
  //       Commands.sequence(
  //           Commands.waitSeconds(SHOOT_SPINUP_SEC),
  //           HopperCommands.pullIn(hopper),
  //           IntakeCommands.pullIn(intake)));
  // }

  private static Command shootFuel(LauncherFOC launcherfoc, Hopper hopper, Intake intake) {
    if (intake == null) {
      return Commands.deadline(
          Commands.waitSeconds(SHOOT_SPINUP_SEC + SHOOT_FEED_SEC),
          LauncherFOCCommands.pullInMid(launcherfoc),
          Commands.sequence(Commands.waitSeconds(SHOOT_SPINUP_SEC), HopperCommands.pullIn(hopper)));
    }

    return Commands.deadline(
        Commands.waitSeconds(SHOOT_SPINUP_SEC + SHOOT_FEED_SEC),
        LauncherFOCCommands.pullInMid(launcherfoc),
        Commands.sequence(
            Commands.waitSeconds(SHOOT_SPINUP_SEC),
            HopperCommands.pullIn(hopper),
            IntakeCommands.pullIn(intake)));
  }
  /** Drives robot-relative for a fixed amount of time. */
  private static Command driveRobotRelative(Drive drive, double vxMetersPerSec, double seconds) {
    return Commands.deadline(
            Commands.waitSeconds(seconds), driveRobotRelativeContinuous(drive, vxMetersPerSec))
        .finallyDo(() -> drive.runVelocity(new ChassisSpeeds()));
  }

  /** Continuously commands a robot-relative forward/backward velocity. */
  private static Command driveRobotRelativeContinuous(Drive drive, double vxMetersPerSec) {
    return Commands.run(
        () -> drive.runVelocity(new ChassisSpeeds(vxMetersPerSec, 0.0, 0.0)), drive);
  }

  private static Command strafeRobotRelative(Drive drive, double vyMetersPerSec, double seconds) {

    return Commands.deadline(
            Commands.waitSeconds(seconds),
            Commands.run(
                () -> drive.runVelocity(new ChassisSpeeds(0.0, vyMetersPerSec, 0.0)), drive))
        .finallyDo(() -> drive.runVelocity(new ChassisSpeeds()));
  }

  /** Stops the drivetrain cleanly. */
  private static Command stopDrive(Drive drive) {
    return Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
