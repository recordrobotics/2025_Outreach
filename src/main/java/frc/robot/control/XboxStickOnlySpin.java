package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class XboxStickOnlySpin extends AbstractControl {

  private final XboxController xbox;
  private final Joystick joystick;
  private static final double SPEED_MULTIPLIER = 0.8;
  private static final double JOYSTICK_DEAD_ZONE = 0.1;

  public XboxStickOnlySpin(int xboxID, int joystickID) {
    this.joystick = new Joystick(joystickID);
    xbox = new XboxController(xboxID);
  }

  @Override
  public DriveCommandData getDriveCommandData() {
    return new DriveCommandData(
        -(getAB().getFirst()) * getDirectionalSpeedLevel(),
        (getAB().getSecond()) * getDirectionalSpeedLevel(),
        (-getSpin() * isJoyMoved() + (getSpinJoy() / 0.7)) * getSpinSpeedLevel(),
        false);
  }

  public int isJoyMoved() {
    if (Math.abs(getSpinJoy()) <= JOYSTICK_DEAD_ZONE) {
      return 1;
    } else {
      return 0;
    }
  }

  public Pair<Double, Double> getXY() {
    double x =
        SimpleMath.ApplyThresholdAndSensitivity(
            xbox.getRawAxis(0), Constants.Control.XBOX_X_THRESHOLD, 1);
    double y =
        SimpleMath.ApplyThresholdAndSensitivity(
            xbox.getRawAxis(1), Constants.Control.XBOX_X_THRESHOLD, 1);
    return super.orientXY(new Pair<Double, Double>(x, y));
  }

  public Pair<Double, Double> getAB() {
    double A =
        SimpleMath.ApplyThresholdAndSensitivity(
            joystick.getX(),
            Constants.Control.JOYSTICK_XY_DEAD_ZONE,
            Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
    double B =
        SimpleMath.ApplyThresholdAndSensitivity(
            joystick.getY(),
            Constants.Control.JOYSTICK_XY_DEAD_ZONE,
            Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
    return super.orientXY(new Pair<Double, Double>(A, B));
  }

  public Double getSpin() {
    return SimpleMath.ApplyThresholdAndSensitivity(
        -xbox.getRawAxis(4),
        Constants.Control.XBOX_SPIN_ROT_THRESHOLD,
        Constants.Control.XBOX_SPIN_ROT_SENSITIVITY);
  }

  public Double getSpinJoy() {
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
    return xbox.getRightTriggerAxis() > 0.3 || joystick.getRawButton(1);
  }

  @Override
  public Boolean getReverse() {
    return xbox.getRawButton(6) || joystick.getRawButton(2);
  }

  @Override
  public Boolean getTwerk() {
    return false; /*
                  || joystick.getRawButton(3)
                  || joystick.getRawButton(4)
                  || joystick.getRawButton(5)
                  || joystick.getRawButton(6); */
    // drivebox.getRawButton(1)
  }
}
