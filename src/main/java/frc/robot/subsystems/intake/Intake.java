/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.SubsystemConstants.intakeName;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.RevRoboticsSubsystem;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

/**
 * This class contains the implementation of the <code>Intake</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2026.0.0
 * @author first.brian Buzzell
 * @version 2026.0.0
 */
public class Intake extends RevRoboticsSubsystem implements ISubsystem {

  /** */
  private final SparkFlex motor;

  private double currentSpeed = 0.0;

  /** Constructs a new instance of the subsystem. */
  public Intake() {
    super(intakeName);
    initConstruction();

    // Create and configure motor
    motor = new SparkFlex(motorCanId, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kCoast);
    // TODO - Configure additional motor parameters from Constants file
    SparkUtil501.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // We start stopped
    currentSpeed = 0;
    setSpeed(currentSpeed);

    finishConstruction();
  }

  // TODO - Document directions for +/- speed
  /**
   * Accepts a manual override of any defaults or PID controlled set points to allow <i>Operator</i>
   * adjustment of the position. Positive values ???? and negative values ????.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  public void acceptTeleopInput(double speed) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }
    currentSpeed = speed;
  }

  /**
   * Sets the desired speed on the motor.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public void disabledInit() {
    // Ensure any motion stops when we go to <i>disabled</code>
    setSpeed(0);
    // TODO - Remember why we don't call motor.stop() vs. setting speed to 0 (or else fix)
  }

  // TODO - Add code to set the default speed on auto & teleop if running that way

  private final String tlmCurrentSpeed = getSubsystem() + "/CurrentSpeed";

  @Override
  public void periodic() {
    setSpeed(currentSpeed);

    Logger.recordOutput(tlmCurrentSpeed, currentSpeed);
    logMotorTelemetry(motor);
  }

  private final String tlmOutput = getSubsystem() + "/Output";

  private final String tlmMotorCurrent = getSubsystem() + "/MotorCurrent";
  private final String tlmMotorTemp = getSubsystem() + "/MotorTemp";
  private final String tlmMotorOvertemp = getSubsystem() + "/MotorOvertemp";

  /**
   * Log the telemetry so it can be recorded and placed on the dashboard. This version is for a
   * <code>Subsystem</code> that has only a <i>motor</i>.
   *
   * @param motor
   */
  private void logMotorTelemetry(SparkBase motor) {
    Logger.recordOutput(tlmOutput, motor.get());

    Logger.recordOutput(tlmMotorCurrent, motor.getOutputCurrent());
    double motorTemp = motor.getMotorTemperature();
    Logger.recordOutput(tlmMotorTemp, motorTemp);
    // Invert so green is OK and red is too hot
    Logger.recordOutput(tlmMotorOvertemp, (!(motorTemp > IntakeConstants.motorOverTemp)));
  }
}
