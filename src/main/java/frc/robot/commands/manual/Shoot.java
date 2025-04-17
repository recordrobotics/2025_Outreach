package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Shoot extends SequentialCommandGroup {

  private static Channel _channel;
  private static Shooter _shooter;

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 0.3; //1.5;
  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 2;

  /**
   * Command that shoots the note into the speaker. Manages all relevant subsystems to do so. 
   * @param channel
   * @param shooter
   */
  public Shoot (Channel channel, Shooter shooter) {
    _channel = channel;
    _shooter = shooter;
    addRequirements(channel);
    addRequirements(shooter);

    final Runnable killSpecified = () -> new KillSpecified(_channel, _shooter);

    addCommands(
        new InstantCommand(()->_shooter.toggle(ShooterStates.SPEAKER), _shooter).handleInterrupt(killSpecified),
        new WaitCommand(flywheelSpinupTime),
        new InstantCommand(()->_channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified),
        new WaitCommand(shootTime),
        new InstantCommand(()-> _shooter.toggle(ShooterStates.OFF), _shooter).handleInterrupt(killSpecified),
        new InstantCommand(()-> _channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified)
    );
  }
}
