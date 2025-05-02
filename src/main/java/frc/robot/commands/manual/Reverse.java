package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Reverse extends Command {

  private static Shooter _acquisition;

  public Reverse(Shooter acquisition) {
    _acquisition = acquisition;
    addRequirements(acquisition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _acquisition.toggle(ShooterStates.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _acquisition.toggle(ShooterStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
