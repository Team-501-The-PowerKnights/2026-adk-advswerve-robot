// // Copyright (c) 2021-2026 Littleton Robotics
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by a BSD
// // license that can be found in the LICENSE file
// // at the root directory of this project.


package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for TalonFX drive motor controller, Spark Max turn motor controller, and
 * duty cycle absolute encoder.
 */
public class ModuleIOSparkHybrid implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;

  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;

  // Closed loop controllers (turn only)
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread (turn only)
  private final Queue<Double> timestampQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean sparkStickyFault = false;

  public ModuleIOSparkHybrid(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    // TalonFX drive (Kraken X60)
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, driveVelocity);
    driveTalon.optimizeBusUtilization();

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.CurrentLimits.SupplyCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.Feedback.SensorToMechanismRatio = driveEncoderPositionFactor;
    driveConfig.Slot0.kP = driveKp;
    driveConfig.Slot0.kD = driveKd;
    driveTalon.getConfigurator().apply(driveConfig);
    driveTalon.setPosition(0.0);

    // SparkMax turn (NEO 550)
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);

    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Odometry queues (turn only)
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Drive (TalonFX)
    drivePosition.refresh();
    driveVelocity.refresh();

    inputs.drivePositionRad = drivePosition.getValueAsDouble();
    inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveTalon.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveTalon.getSupplyCurrent().getValueAsDouble();

    inputs.driveConnected = driveConnectedDebounce.calculate(driveTalon.isConnected());

    // Turn (SparkMax)
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Odometry (timestamps + turn; drive from TalonFX as a single-sample array)
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = new double[] {drivePosition.getValueAsDouble()};
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(new VoltageOut(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveTalon.setControl(new VelocityVoltage(velocityRadPerSec).withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
