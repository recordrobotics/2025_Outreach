package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class Xbox extends AbstractControl {

  private XboxController drivebox;
  private double speed_level = 0.8;

  public Xbox(int driveboxID, int notesboxID) {
    // Sets up xbox controllers
    drivebox = new XboxController(driveboxID);
  }

  // Joystick joystick = new Joystick(1);

  @Override
  public DriveCommandData getDriveCommandData() {
    // Gets information needed to drive
    DriveCommandData driveCommandData =
        new DriveCommandData(
            -(getXY().getFirst()) * getDirectionalSpeedLevel(),
            (getXY().getSecond()) * getDirectionalSpeedLevel(),
            (-getSpin()) * getSpinSpeedLevel(),
            false);

    // Returns
    return driveCommandData;
  }


  public Pair<Double, Double> getXY() {
    double X =
        SimpleMath.ApplyThresholdAndSensitivity(
            drivebox.getRawAxis(0),
            Constants.Control.XBOX_X_THRESHOLD,
            Constants.Control.XBOX_DIRECTIONAL_SENSITIVITY);
    double Y =
        SimpleMath.ApplyThresholdAndSensitivity(
            drivebox.getRawAxis(1),
            Constants.Control.XBOX_Y_THRESHOLD,
            Constants.Control.XBOX_DIRECTIONAL_SENSITIVITY);
    return super.OrientXY(new Pair<Double, Double>(X, Y));
  }



  public Double getSpin() {
    return SimpleMath.ApplyThresholdAndSensitivity(
        -drivebox.getRawAxis(4),
        Constants.Control.XBOX_SPIN_ROT_THRESHOLD,
        Constants.Control.XBOX_SPIN_ROT_SENSITIVITY);
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
    return drivebox.getRightTriggerAxis() > 0.3;
  }

  @Override
  public Boolean getReverse() {
    return drivebox.getRawButton(6);
  }

  @Override
  public Boolean getLoad() {
    return drivebox.getRawButton(1);
  }
}
