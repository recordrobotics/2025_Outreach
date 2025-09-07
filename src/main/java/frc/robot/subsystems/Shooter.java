package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Shooter extends KillableSubsystem {
  private TalonFX flywheel =
      new TalonFX(RobotMap.Shooter.FLYWHEEL_MOTOR_DEVICE_ID); // changed to kraken
  private final VoltageOut voltageOut = new VoltageOut(0);

  public Shooter() {
    toggle(ShooterStates.OFF);
    // // ShuffleboardUI.Test.addSlider("Flywheel", flywheel.get(), -1, 1).subscribe(flywheel::set);
    ShuffleboardUI.Test.addSlider("Flywheel", 0, -12, 12)
        .subscribe(volts -> flywheel.setControl(voltageOut.withOutput(volts)));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.Shooter.GEAR_RATIO;

    flywheel
        .getConfigurator()
        .apply(
            config
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));
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
