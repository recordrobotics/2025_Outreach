package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Shooter extends KillableSubsystem {
  private Spark flywheel =
      new Spark(RobotMap.Shooter.FLYWHEEL_MOTOR_DEVICE_ID); // old PWM Spark (confusing)

  public Shooter() {
    toggle(ShooterStates.OFF);
    ShuffleboardUI.Test.addSlider("Flywheel", flywheel.get(), -1, 1).subscribe(flywheel::set);
  }

  public enum ShooterStates {
    SPEAKER,
    AMP,
    REVERSE,
    OFF;
  }

  public void toggle(double speed) {
    flywheel.set(speed);
  }

  public void toggle(ShooterStates state) {
    switch (state) {
      case SPEAKER:
        toggle(Constants.Shooter.SPEAKER_SPEED);
        break;
      case AMP:
        toggle(Constants.Shooter.AMP_SPEED);
        break;
      case REVERSE:
        toggle(Constants.Shooter.REVERSE_SPEED);
        break;
      default:
        toggle(0);
        break;
    }
  }

  @Override
  public void kill() {
    toggle(ShooterStates.OFF);
  }
}
