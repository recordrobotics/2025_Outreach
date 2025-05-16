package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.utils.DriveCommandData;
import frc.robot.subsystems.Drivetrain;
import frc.robot.control.AbstractControl;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;


public class RobotTwerk extends SequentialCommandGroup {

    private Drivetrain _drivetrain;

    public AbstractControl _controls;
  
  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 0.3; // 1.5;

  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double TwerkTime = 0.5;
  private double TwerkDistance = 0.5; // meters
    private final int RepeatTime = 2;
  
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
          for(int i =0; i < RepeatTime*2; i++) {
              TwerkDistance = -TwerkDistance;
            addCommands(

            new InstantCommand(() -> _drivetrain.drive(new DriveCommandData(TwerkDistance,0,0, false)), _drivetrain)
            .handleInterrupt(killSpecified),
            // new WaitCommand(flywheelSpinupTime),
            new WaitCommand(TwerkTime)
            );
        }

    addCommands(


    new InstantCommand(() -> _drivetrain.drive(new DriveCommandData(0,0,0, false)), _drivetrain)
    .handleInterrupt(killSpecified));
  }
}
