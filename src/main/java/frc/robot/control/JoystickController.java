package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;


public class JoystickController extends AbstractControl {

  private double speed_level = 0.8;
  Joystick drivestick;
  public JoystickController(int drivestickID) {
    // Sets up xbox controllers
    drivestick = new Joystick(drivestickID);
  }

  @Override
  public DriveCommandData getDriveCommandData() {
    // Gets information needed to drive
    DriveCommandData driveCommandData =
        new DriveCommandData(
            -(getAB().getFirst()) * getDirectionalSpeedLevel(),
            (getAB().getSecond()) * getDirectionalSpeedLevel(),
            (-(getSpinJoy() / 0.7)) * getSpinSpeedLevel(),
            false);

    // Returns
    return driveCommandData;
  }



  public Pair<Double, Double> getAB() {
    double A =
        SimpleMath.ApplyThresholdAndSensitivity(
          drivestick.getX(),
            Constants.Control.JOYSTICK_X_THRESHOLD,
            Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
    double B =
        SimpleMath.ApplyThresholdAndSensitivity(
          drivestick.getY(),
            Constants.Control.JOYSTICK_Y_THRESHOLD,
            Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY);
    return super.OrientXY(new Pair<Double, Double>(A, B));
  }


  public Double getSpinJoy() {
    return SimpleMath.ApplyThresholdAndSensitivity(
        -drivestick.getTwist(),
        Constants.Control.JOYSTICK_SPIN_THRESHOLD,
        Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
  }

  public Double getDirectionalSpeedLevel() {
    return speed_level;
  }

  public Double getSpinSpeedLevel() {
    return .7 * speed_level; // 3.14
  }

  // @Override
  // public Boolean getPoseReset() {
  //     return drivebox.getRawButtonPressed(7);
  // }

  // @Override
  // public Boolean getKillAuto() {
  //     return drivebox.getRawButton(8);
  // }

  @Override
  public Boolean getShoot() {
    return drivestick.getRawButton(1);
  }

  @Override
  public Boolean getReverse() {
    return drivestick.getRawButton(2);
  }

  @Override
  public Boolean getLoad() {
    return drivestick.getRawButton(3)
        || drivestick.getRawButton(4)
        || drivestick.getRawButton(5)
        || drivestick.getRawButton(6);
  }
    
}
