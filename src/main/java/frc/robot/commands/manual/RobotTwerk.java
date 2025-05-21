package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.KillSpecified;
import frc.robot.control.AbstractControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;

public class RobotTwerk extends SequentialCommandGroup {

  private Drivetrain _drivetrain;

  public AbstractControl _controls;

  /** Number of seconds it takes for the flywheel to spin up */
  // private final double flywheelSpinupTime = 0.3; // 1.5;

  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double TwerkTime = Constants.Twerk.TWERK_TIME;

  // private double TwerkSpeed = Constants.Twerk.TWERK_SPEED; // meters
  private final int RepeatTime = Constants.Twerk.REPEATS;

  /**
   * Command that shoots the note into the speaker. Manages all relevant subsystems to do so.
   *
   * @param channel
   * @param shooter
   */
  public RobotTwerk(Drivetrain drivetrain) {
    _drivetrain = drivetrain;
    addRequirements(drivetrain);

    final Runnable killSpecified = () -> new KillSpecified(_drivetrain);
    for (int i = 0; i < RepeatTime * 2; i++) {
      double TwerkSpeed;
      if (i % 2 == 0) {
        TwerkSpeed = Constants.Twerk.TWERK_SPEED;
      } else {
        TwerkSpeed = -Constants.Twerk.TWERK_SPEED;
      }
      // TwerkSpeed = -TwerkSpeed;
      addCommands(
          new InstantCommand(
                  () -> _drivetrain.drive(new DriveCommandData(TwerkSpeed, 0, 0, false)),
                  _drivetrain)
              .handleInterrupt(killSpecified),
          // new WaitCommand(flywheelSpinupTime),
          new WaitCommand(TwerkTime));
    }

    addCommands(
        new InstantCommand(
                () -> _drivetrain.drive(new DriveCommandData(0, 0, 0, false)), _drivetrain)
            .handleInterrupt(killSpecified));
  }
}
