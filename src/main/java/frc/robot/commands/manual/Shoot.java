package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Shoot extends SequentialCommandGroup {

  private static Shooter _shooter;

  public Shoot(Shooter shooter) {
    _shooter = shooter;
    addRequirements(shooter);

    final Runnable killSpecified = () -> new KillSpecified(_shooter);
    System.out.println("shoot");
    addCommands(
        new InstantCommand(() -> _shooter.toggle(ShooterStates.SPEAKER), _shooter)
            .handleInterrupt(killSpecified),
        new WaitCommand(Constants.Shooter.SHOOT_TIME),
        new InstantCommand(() -> _shooter.toggle(ShooterStates.OFF), _shooter)
            .handleInterrupt(killSpecified));
  }
}
