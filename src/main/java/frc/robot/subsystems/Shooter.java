package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Shooter extends KillableSubsystem {
  private TalonFX flywheel =
      new TalonFX(RobotMap.Shooter.FLYWHEEL_MOTOR_DEVICE_ID); // old PWM Spark (confusing)
      private final VoltageOut voltageOut = new VoltageOut(0);


      public Shooter() {
        toggle(ShooterStates.OFF);
        // ShuffleboardUI.Test.addSlider("Flywheel", flywheel.get(), -1, 1).subscribe(flywheel::set);
        ShuffleboardUI.Test
            .addSlider("Flywheel", 0, -12, 12)
            .subscribe(volts -> flywheel.setControl(voltageOut.withOutput(volts)));
      }
    
      public enum ShooterStates {
        SPEAKER,
        AMP,
        REVERSE,
        OFF;
      }
    
      public void toggle(double volts) {
        // flywheel.set(speed);
        flywheel.setControl(voltageOut.withOutput(volts));
    
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



