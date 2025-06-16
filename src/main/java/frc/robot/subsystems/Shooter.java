package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Shooter extends KillableSubsystem {
  private Talon flywheel =
      new Talon(RobotMap.Shooter.FLYWHEEL_MOTOR_DEVICE_ID); // old PWM Spark (confusing)

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

  public void toggle(double volt) {
    flywheel.setVoltage(volt);
  }

  public void toggle(ShooterStates state) {
    switch (state) {
      case SPEAKER:
        toggle(Constants.Shooter.SPEAKER_VOLTAGE);
        break;
      case REVERSE:
        toggle(Constants.Shooter.REVERSE_VOLTAGE);
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
