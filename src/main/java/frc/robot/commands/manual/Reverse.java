package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Reverse extends Command {

  private static Shooter _acquisition;
  private static Channel _channel;

  public Reverse (Shooter acquisition, Channel channel) {
    _acquisition = acquisition;
    _channel = channel;
    addRequirements(acquisition);
    addRequirements(channel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _acquisition.toggle(ShooterStates.REVERSE);
    _channel.toggle(ChannelStates.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _acquisition.toggle(ShooterStates.OFF);
    _channel.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}