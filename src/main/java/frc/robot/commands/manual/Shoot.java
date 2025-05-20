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

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 1.5; // 1.5;

  /** Number of seconds it takes to shoot once the flywheel h as been spun up */

  /**
   * Command that shoots the note into the speaker. Manages all relevant subsystems to do so.
   *
   * @param channel
   * @param shooter
   */
  public Shoot(Shooter shooter) {
    _shooter = shooter;
    addRequirements(shooter);

    final Runnable killSpecified = () -> new KillSpecified(_shooter);
    System.out.println("shoot");
    addCommands(
        new InstantCommand(() -> _shooter.toggle(ShooterStates.SPEAKER), _shooter)
            .handleInterrupt(killSpecified),
        // new WaitCommand(flywheelSpinupTime),
        new WaitCommand(Constants.Shooter.SHOOT_TIME),
        new InstantCommand(() -> _shooter.toggle(ShooterStates.OFF), _shooter)
            .handleInterrupt(killSpecified));
  }
}
