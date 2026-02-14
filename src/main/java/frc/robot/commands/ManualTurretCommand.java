package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import java.util.function.DoubleSupplier;

public class ManualTurretCommand extends Command {
  private final Turret turret;
  private final DoubleSupplier output; // -1..+1 scaled inside constants if desired

  public ManualTurretCommand(Turret turret, DoubleSupplier output) {
    this.turret = turret;
    this.output = output;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double val = output.getAsDouble();
    // Clamp just in case
    if (val > 1.0) val = 1.0;
    if (val < -1.0) val = -1.0;

    turretMotorSet(val);
  }

  private void turretMotorSet(double val) {
    // Convert -1..1 into actual duty cycle using the constant
    turret.stop();
    if (val > 0.05) {
      // right
      // scale to our manual duty cycle
      turret.turnRight();
    } else if (val < -0.05) {
      // left
      turret.turnLeft();
    } else {
      turret.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted (button released)
  }
}
