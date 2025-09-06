package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class JoystickController extends AbstractControl {

  private static final double SPEED_MULTIPLIER = 0.8;
  private final Joystick joystick;

  public JoystickController(int joystickID) {
    // Sets up xbox controllers
    joystick = new Joystick(joystickID);
  }

  @Override
  public DriveCommandData getDriveCommandData() {
    return new DriveCommandData(
        -(getXY().getFirst()) * getDirectionalSpeedLevel(),
        (getXY().getSecond()) * getDirectionalSpeedLevel(),
        (-(getSpin() / 0.7)) * getSpinSpeedLevel(),
        false);
  }

  public Pair<Double, Double> getXY() {
    double x =
        SimpleMath.ApplyThresholdAndSensitivity(
            joystick.getX(),
            Constants.Control.JOYSTICK_XY_DEAD_ZONE,
            Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
    double y =
        SimpleMath.ApplyThresholdAndSensitivity(
            joystick.getY(),
            Constants.Control.JOYSTICK_XY_DEAD_ZONE,
            Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
    return super.orientXY(new Pair<Double, Double>(x, y));
  }

  public Double getSpin() {
    return SimpleMath.ApplyThresholdAndSensitivity(
        -joystick.getTwist(),
        Constants.Control.JOYSTICK_SPIN_THRESHOLD,
        Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
  }

  public Double getDirectionalSpeedLevel() {
    return SPEED_MULTIPLIER;
  }

  public Double getSpinSpeedLevel() {
    return .7 * SPEED_MULTIPLIER; // 3.14
  }

  @Override
  public Boolean getShoot() {
    return joystick.getRawButton(1);
  }

  @Override
  public Boolean getReverse() {
    return joystick.getRawButton(2);
  }

  @Override
  public Boolean getTwerk() {
    return false; /*drivestick.getRawButton(3)
                  || drivestick.getRawButton(4)
                  || drivestick.getRawButton(5)
                  || drivestick.getRawButton(6);*/
  }
}
