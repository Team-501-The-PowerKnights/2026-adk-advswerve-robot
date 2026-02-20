// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

/**
 * Module IO for a "hybrid" REV swerve: - Drive motor: CTRE TalonFX / Kraken X60 (Phoenix 6) - Turn
 * motor + absolute encoder: REV Spark Max + built-in absolute encoder
 *
 * <p>This lets you keep the REV swerve pod + turning hardware while swapping only the drive
 * motor/controller.
 */
public class ModuleIOKrakenDriveSparkTurn implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;

  private final boolean driveInverted = false; // Set based on your wiring/gearbox

  // Turn closed loop controller (REV)
  private final SparkClosedLoopController turnController;

  // Phoenix controls (re-used objects to avoid allocations)
  private final VoltageOut driveVoltageOut = new VoltageOut(0.0);
  private final VelocityVoltage driveVelocityVoltage = new VelocityVoltage(0.0);

  // Phoenix status signals
  private final com.ctre.phoenix6.StatusSignal<Angle> driveRotorPosition;
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> driveRotorVelocity;
  private final com.ctre.phoenix6.StatusSignal<Voltage> driveMotorVoltage;
  private final com.ctre.phoenix6.StatusSignal<Current> driveStatorCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Used by SparkUtil.ifOk
  private boolean sparkStickyFault = false;

  public ModuleIOKrakenDriveSparkTurn(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    // --- Drive: Kraken X60 (TalonFX) ---
    int driveCanId =
        switch (module) {
          case 0 -> frontLeftDriveCanId;
          case 1 -> frontRightDriveCanId;
          case 2 -> backLeftDriveCanId;
          case 3 -> backRightDriveCanId;
          default -> 0;
        };
    driveTalon = new TalonFX(driveCanId);

    // Phoenix signals
    driveRotorPosition = driveTalon.getPosition(); // rotations
    driveRotorVelocity = driveTalon.getVelocity(); // rotations/sec
    driveMotorVoltage = driveTalon.getMotorVoltage(); // volts
    driveStatorCurrent = driveTalon.getStatorCurrent(); // amps

    // Configure TalonFX
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Neutral / invert (use your existing driveInverted constant)
    MotorOutputConfigs motorOut = new MotorOutputConfigs();
    motorOut.NeutralMode = NeutralModeValue.Brake;
    motorOut.Inverted =
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput = motorOut;

    // Current limits (treat existing driveMotorCurrentLimit as supply/stator limit starter)
    CurrentLimitsConfigs current = new CurrentLimitsConfigs();
    current.SupplyCurrentLimitEnable = true;
    current.SupplyCurrentLimit = driveMotorCurrentLimit;
    current.StatorCurrentLimitEnable = true;
    current.StatorCurrentLimit = driveMotorCurrentLimit;
    cfg.CurrentLimits = current;

    // Slot0 PID (start with same Kp/Kd values; you'll likely retune)
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = driveKp;
    slot0.kD = driveKd;
    cfg.Slot0 = slot0;

    // Apply config
    driveTalon.getConfigurator().apply(cfg);

    // Set status signal update rates (keep it light; odometry is handled in code)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, driveRotorPosition, driveRotorVelocity); // match odometryFrequency-ish
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveMotorVoltage, driveStatorCurrent);

    // --- Turn: REV Spark Max + absolute encoder (unchanged from template) ---
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

    turnEncoder = ((SparkMax) turnSpark).getAbsoluteEncoder();
    turnController = ((SparkMax) turnSpark).getClosedLoopController();

    // Configure turn motor (same as ModuleIOSpark)
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
        .positionWrappingMinInput(turnPIDMinInput)
        .positionWrappingMaxInput(turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
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
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // --- Drive (Phoenix) ---
    StatusCode status =
        BaseStatusSignal.refreshAll(
            driveRotorPosition, driveRotorVelocity, driveMotorVoltage, driveStatorCurrent);

    boolean driveOk = status.isOK();
    double rotorPosRot = driveRotorPosition.getValueAsDouble();
    double rotorVelRps = driveRotorVelocity.getValueAsDouble();

    // Convert rotor -> wheel radians
    inputs.drivePositionRad = rotorPosRot * (2.0 * Math.PI) / driveMotorReduction;
    inputs.driveVelocityRadPerSec = rotorVelRps * (2.0 * Math.PI) / driveMotorReduction;
    inputs.driveAppliedVolts = driveMotorVoltage.getValueAsDouble();
    inputs.driveCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveConnected = driveConnectedDebounce.calculate(driveOk);

    // --- Turn (Spark) ---
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new java.util.function.DoubleSupplier[] {
          turnSpark::getAppliedOutput, turnSpark::getBusVoltage
        },
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // --- Odometry samples ---
    // To keep this drop-in simple, we publish a single sample per cycle.
    // (The stock template uses SparkOdometryThread to get a higher-rate queue.)
    //    double now = Timer.getFPGATimestamp();
    double now = Timer.getTimestamp();
    inputs.odometryTimestamps = new double[] {now};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveTalon.setControl(driveVoltageOut.withOutput(volts));
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnSpark.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // Same feedforward structure as template, but applied as a velocity+FF voltage request.
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;

    // Convert wheel rad/s -> rotor rotations/s
    double rotorRps = (velocityRadPerSec * driveMotorReduction) / (2.0 * Math.PI);

    driveTalon.setControl(driveVelocityVoltage.withVelocity(rotorRps).withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
