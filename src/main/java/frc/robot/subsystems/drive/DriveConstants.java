// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int pigeonCanId = 10;

  public static final int frontLeftDriveCanId = 11;
  public static final int backLeftDriveCanId = 12;
  public static final int frontRightDriveCanId = 13;
  public static final int backRightDriveCanId = 14;

  public static final int frontLeftTurnCanId = 21;
  public static final int backLeftTurnCanId = 22;
  public static final int frontRightTurnCanId = 23;
  public static final int backRightTurnCanId = 24;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction =
      (45.0 * 20.0) / (15.0 * 15.0); // MAXSwerve with 15 pinion teeth  // and 20 spur teeth
  // REV Gear Ratios:
  // Current REV Gear Ratios: Upgrade Extra High 3 (4.00:1) with 15T pinion and 20T spur
  /* https://www.revrobotics.com/rev-21-3008/
  | Kit     | Speed Option | Gear Ratio | Free Speed (NEO Vortex) | Pinion Teeth | Spur Teeth |
  |---------|--------------|------------|-------------------------|--------------|------------|
  |  Base   | Low          | 5.50:1     | 4.92 m/s (16.15 ft/s)   | 12T (14)     | 22T        |
  |  Base   | Medium       | 5.08:1     | 5.33 m/s (17.49 ft/s)   | 13T (14)     | 22T        |
  |  Base   | High         | 4.71:1     | 5.74 m/s (18.84 ft/s)   | 14T          | 22T        |
  |  Base   | Speed Option | Gear Ratio | Free Speed (NEO Vortex) | Pinion Teeth | Spur Teeth |
  |---------|--------------|------------|-------------------------|--------------|------------|
  | Upgrade | Extra High 1 | 4.50:1     | 6.01 m/s (19.73 ft/s)   | 14T          | 21T (22)   |
  | Upgrade | Extra High 2 | 4.29:1     | 6.32 m/s (20.72 ft/s)   | 14T          | 20T (22)   |
  | Upgrade | Extra High 3 | 4.00:1     | 6.77 m/s (22.20 ft/s)   | 15T (16)     | 20T        |
  | Upgrade | Extra High 4 | 3.75:1     | 7.22 m/s (23.68 ft/s)   | 16T          | 20T        |
  | Upgrade | Extra High 5 | 3.56:1     | 7.60 m/s (24.93 ft/s)   | 16T          | 19T (20)   |
  */

  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
