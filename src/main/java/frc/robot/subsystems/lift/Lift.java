/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.lift;

import static frc.robot.subsystems.SubsystemConstants.liftName;
import static frc.robot.subsystems.lift.LiftConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.RevRoboticsSubsystem;
import frc.robot.util.SparkUtil501;
import java.text.DecimalFormat;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class contains the implementation of the <i>Intake</i> <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.brian Buzzell
 * @version 2026.0.0
 */
public class Lift extends RevRoboticsSubsystem implements ISubsystem {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    // RESTORE PID
    // PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  /** Enumeration of set positions */
  public enum Task {
    //
    // Pose for raised and out of the way
    RAISED_POSE("Raised_Pose", 0.0),
    // Pose for down and picking up balls
    GROUND_POSE("Ground_Pose", 0.0),
    // Position for 'homing' during match
    HOME("Home", LiftConstants.minHeight),
    // Position for starting match
    START("Start", LiftConstants.minHeight),
    // Special case of current position when enabled
    HOLD("Hold", 0.0);

    private final String name;
    private double target;

    Task(String name, double target) {
      this.name = name;
      this.target = target;
    }

    public String getName() {
      return name;
    }

    public double getTarget() {
      return this.target;
    }

    public void setTarget(double target) {
      if (this.getName().equals("Hold")) {
        this.target = target;
      } else {
        // TODO - Add a logged error here
      }
    }
  }

  // Flag for whether first periodic() has run
  private boolean firstPeriodic;

  // Current mode
  private Mode currentMode;
  // Current task
  private Task currentTask;
  // If manual mode - then the current setting
  private double currentSpeed;
  // If PID mode - then the current setting
  private double currentTarget;

  // Persistent encoder init stuff (so can be logged)
  private final StringBuilder encoderInitBuf;
  // Persistent PID tuning stuff (so can be logged)
  private final StringBuilder pidConfigBuf;

  // AdvantageKit editiable numbers for tuning on dashboard
  private final LoggedNetworkNumber pidP;
  private final LoggedNetworkNumber pidI;
  private final LoggedNetworkNumber pidD;

  /** */
  private final SparkFlex motor;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  /** */
  // private final SparkFlex follower;

  /** Constructs a new instance of the subsystem. */
  public Lift() {
    super(liftName);
    initConstruction();

    firstPeriodic = false;

    encoderInitBuf = new StringBuilder();
    pidConfigBuf = new StringBuilder();

    // Create and configure motor
    motor = new SparkFlex(motorCanId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig
        .inverted(LiftConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(LiftConstants.motorCurrentLimit)
        .voltageCompensation(LiftConstants.motorVoltageComp)
        .softLimit
        // .forwardSoftLimitEnabled(false)
        // .reverseSoftLimitEnabled(false);
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(LiftConstants.maxHeight)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(LiftConstants.minHeight);
    motorConfig.absoluteEncoder.inverted(LiftConstants.encoderInverted);
    motorConfig.encoder.positionConversionFactor(LiftConstants.gearRatio);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(LiftConstants.pidMaxNegOut, LiftConstants.pidMaxPosOut)
        .pid(LiftConstants.pidKp, LiftConstants.pidKi, LiftConstants.pidKd);

    // TODO - Configure additional motor parameters from Constants file
    SparkUtil501.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Initialize encoder based on absolute
    {
      double absEncoderPos = motor.getAbsoluteEncoder().getPosition();
      double absEncoderPosScaled = absEncoderPos * LiftConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      System.out.println("Lift: initial encoder values = " + collectEncoderValues());

      // Startup in PID at current location
      holdAtPositionWithPID(absEncoderPosScaled);
    }

    // Put PID tuning on dashboard
    StringBuilder buf = new StringBuilder("/Tuning/").append(getName());
    pidP = new LoggedNetworkNumber(buf.toString() + "/1_pid_P", LiftConstants.pidKp);
    pidP.set(LiftConstants.pidKp);
    pidI = new LoggedNetworkNumber(buf.toString() + "/2_pid_I", LiftConstants.pidKi);
    pidI.set(LiftConstants.pidKi);
    pidD = new LoggedNetworkNumber(buf.toString() + "/3_pid_D", LiftConstants.pidKd);
    pidD.set(LiftConstants.pidKd);
    //
    System.out.println("Lift: initial PID values = " + collectPIDValues());

    finishConstruction();
  }

  /**
   * Sets the subsystem to use the current position with PID control.
   *
   * @param position - Encoder position to use
   */
  private void holdAtPositionWithPID(double position) {
    // RESTORE PID
    // // Using PID at current location
    // currentMode = Mode.PID;
    // // Use task of Joystick
    // Task.HOLD.setTarget(position);
    // setTask(Task.HOLD);
    // // no (manual) speed control
    // currentSpeed = 0.0;
  }

  private String collectEncoderValues() {
    encoderInitBuf.setLength(0);

    double absEncoderPos = motor.getAbsoluteEncoder().getPosition();
    double absEncoderPosScaled = absEncoderPos * LiftConstants.gearRatio;
    double relEncoderPos = encoder.getPosition();

    DecimalFormat df = new DecimalFormat("0.00000");

    encoderInitBuf.append("absEncoder = ").append(df.format(absEncoderPos));
    encoderInitBuf.append(", scaled = ").append(df.format(absEncoderPosScaled));
    encoderInitBuf.append(", relEncoder = ").append(df.format(relEncoderPos));

    return encoderInitBuf.toString();
  }

  private String collectPIDValues() {
    pidConfigBuf.setLength(0);

    DecimalFormat df = new DecimalFormat("0.00000");

    double curP = motor.configAccessor.closedLoop.getP();
    double curI = motor.configAccessor.closedLoop.getI();
    double curD = motor.configAccessor.closedLoop.getD();

    pidConfigBuf.append("P=").append(df.format(curP));
    pidConfigBuf.append(" [").append(df.format(pidP.get())).append("]");
    pidConfigBuf.append(",  ");
    pidConfigBuf.append("I=").append(df.format(curI));
    pidConfigBuf.append(" [").append(df.format(pidI.get())).append("]");
    pidConfigBuf.append(",  ");
    pidConfigBuf.append("D=").append(df.format(curD));
    pidConfigBuf.append(" [").append(df.format(pidD.get())).append("]");

    return pidConfigBuf.toString();
  }

  @Override
  public void teleopInit() {
    if (LiftConstants.doPidTuning) {
      System.out.println("Lift::teleopInit: " + collectPIDValues());
    }

    // Set the PID target to be the current position so it doesn't move
    // Restore PID
    // holdAtPositionWithPID(getPosition());
  }

  @Override
  public void teleopExit() {
    if (LiftConstants.doPidTuning) {
      SparkFlexConfig config = new SparkFlexConfig();
      config.closedLoop.pid(pidP.get(), pidI.get(), pidD.get());

      SparkUtil501.tryUntilOk(
          motor,
          5,
          () ->
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

      System.out.println("Lift::teleopExit: " + collectPIDValues());
    }
  }

  /**
   * Accepts a <code>Task</code> which defines a set point target to use for PID control of the
   * position.
   *
   * @param task - The task to set.
   */
  public void setTask(Task task) {
    currentTask = task;
    currentTarget = task.getTarget();
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
   * motor. Preferred method of stopping the motor - especially if running some kind of PID control.
   */
  public void stop() {
    // Make the subsystem state correct
    currentSpeed = 0;
    currentMode = Mode.MANUAL;
    // Use stop to ensure it stops no matter what control mode is in play
    motor.stopMotor();
  }

  // TODO - Document directions for +/- speed
  /**
   * Accepts a manual override of any defaults or PID controlled set points to allow <i>Operator</i>
   * adjustment of the position. Positive values ???? and negative values ????.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  public void acceptInput(double speed) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    currentSpeed = speed;

    // if (speed == 0) {
    //   // No joystick input (so either revert to PID or ignore if currently PID)
    //   if (currentMode == Mode.MANUAL) {
    //     // Use current position for hold point
    //     holdAtPositionWithPID(getPosition());
    //   }
    // } else {
    //   // Valid teleop inputs (so either switch to MANUAL or just update speed)
    //   if (currentMode == Mode.PID) {
    //     currentMode = Mode.MANUAL;
    //   }
    // }
    currentMode = Mode.MANUAL;
  }

  /**
   * Gets the current <code>encoder</code> position. This method should be used everywhere in this
   * class to get the value.
   *
   * @return current encoder position
   */
  private double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Sets the desired speed on the motor.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  private void setSpeed(double speed) {
    controller.setSetpoint(speed, ControlType.kDutyCycle);
  }

  /**
   * Sets the controller to use a PID-based position reference.
   *
   * @param position
   */
  private void setTarget(double position) {
    controller.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public void disabledInit() {
    // Ensure any motion stops when we go to <i>disabled</code>
    setSpeed(0);
  }

  // TODO - Add code to set the default speed on auto & teleop if running that way

  private final String tlmCurrentSpeed = getSubsystem() + "/CurrentSpeed";

  @Override
  public void periodic() {
    if (!firstPeriodic) {
      collectEncoderValues();
      firstPeriodic = true;
    }

    if (currentMode == Mode.MANUAL) {
      setSpeed(currentSpeed);
    } else {
      setTarget(currentTarget);
      // setSpeed(0);
    }

    Logger.recordOutput(tlmCurrentSpeed, currentSpeed);
    logMotorTelemetry(motor);
    logPIDTelemetry();
  }

  private final String tlmCurrentMode = getSubsystem() + "/CurrentMode";
  private final String tlmIsPid = getSubsystem() + "/IsPid";
  private final String tlmCurrentTask = getSubsystem() + "/CurrentTask";
  private final String tlmCurrentTarget = getSubsystem() + "/CurrentTarget";
  private final String tlmPosition = getSubsystem() + "/Position";
  private final String tlmEncoderConfig = getSubsystem() + "/EncoderConfig";
  private final String tlmDoPidTuning = getSubsystem() + "/DoPidTuning";
  private final String tlmPidConfig = getSubsystem() + "/PidConfig";

  /**
   * Log the telemetry so it can be recorded and placed on the dashboard. This version is for a
   * <code>Subsystem</code> that has a <i>position PID</i>.
   */
  private void logPIDTelemetry() {
    // Logger.recordOutput(tlmCurrentMode, currentMode.name());
    // //    Logger.recordOutput(tlmIsPid, (currentMode == Mode.PID));
    // //    Logger.recordOutput(tlmCurrentTask, currentTask.getName());
    // Logger.recordOutput(tlmCurrentTarget, currentTarget);
    // Logger.recordOutput(tlmPosition, getPosition());
    // Logger.recordOutput(tlmEncoderConfig, encoderInitBuf.toString());
    // Logger.recordOutput(tlmDoPidTuning, LiftConstants.doPidTuning);
    // Logger.recordOutput(tlmPidConfig, pidConfigBuf.toString());
  }

  private final String tlmOutput = getSubsystem() + "/Output";

  private final String tlmMotorCurrent = getSubsystem() + "/MotorCurrent";
  private final String tlmMotorTemp = getSubsystem() + "/MotorTemp";
  private final String tlmMotorOvertemp = getSubsystem() + "/MotorOvertemp";

  /**
   * Log the telemetry so it can be recorded and placed on the dashboard. This version is for a
   * <code>Subsystem</code> that has a <i>motor</i> and a <i>follower</i>.
   *
   * @param motor
   * @param follower
   */
  private void logMotorTelemetry(SparkBase motor) {
    Logger.recordOutput(tlmOutput, motor.get());

    Logger.recordOutput(tlmMotorCurrent, motor.getOutputCurrent());
    double motorTemp = motor.getMotorTemperature();
    Logger.recordOutput(tlmMotorTemp, motorTemp);
    // Invert so green is OK and red is too hot
    Logger.recordOutput(tlmMotorOvertemp, (!(motorTemp > LiftConstants.motorOverTemp)));
  }
}
