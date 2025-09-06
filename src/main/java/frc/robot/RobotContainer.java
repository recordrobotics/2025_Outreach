package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.manual.*;
import frc.robot.control.*;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Shooter shooter = new Shooter();

  public RobotContainer() {
    ShuffleboardUI.Overview.addControls(
        new XboxControl(Constants.ID.XBOX_ID),
        new XboxOnlySpin(Constants.ID.XBOX_ID),
        new JoystickController(Constants.ID.JOYSTICK_ID),
        new XboxStick(Constants.ID.XBOX_ID, Constants.ID.JOYSTICK_ID),
        new XboxStickOnlySpin(Constants.ID.XBOX_ID, Constants.ID.JOYSTICK_ID));

    configureButtonBindings();
  }

  public void teleopInit() {
    drivetrain.setDefaultCommand(new ManualDrive(drivetrain));
  }

  private static void configureButtonBindings() {
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getReverse())
        .toggleOnTrue(new Reverse(shooter));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShoot())
        .toggleOnTrue(new Shoot(shooter));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getTwerk())
        .toggleOnTrue(new RobotTwerk(drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void testPeriodic() {
    ShuffleboardUI.Test.testPeriodic();
  }
}
