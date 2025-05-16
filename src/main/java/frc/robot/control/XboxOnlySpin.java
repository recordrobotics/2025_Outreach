package frc.robot.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.SimpleMath;

public class XboxOnlySpin extends AbstractControl {

  private XboxController drivebox;
  private double speed_level = 0.8;

  public XboxOnlySpin(int driveboxID) {
    // Sets up xbox controller
    drivebox = new XboxController(driveboxID);
  }

  private Translation3d currentXYR = new Translation3d();

  @Override
  public DriveCommandData getDriveCommandData() {

    Translation3d xyr = new Translation3d(0, 0, (-getSpin()) * getSpinSpeedLevel());
    var xy = MathUtil.slewRateLimit(currentXYR.toTranslation2d(), xyr.toTranslation2d(), 0.02, 2.0);
    var r =
        MathUtil.slewRateLimit(
            new Translation2d(currentXYR.getZ(), 0), new Translation2d(xyr.getZ(), 0), 0.02, 7.0);

    currentXYR = new Translation3d(xy.getX(), xy.getY(), r.getX());

    // Gets information needed to drive
    DriveCommandData driveCommandData =
        new DriveCommandData(currentXYR.getX(), currentXYR.getY(), currentXYR.getZ(), false);

    // Returns
    return driveCommandData;
  }

  public Double getSpin() {
    return SimpleMath.ApplyThresholdAndSensitivity(
        drivebox.getRawAxis(4),
        Constants.Control.XBOX_SPIN_ROT_THRESHOLD,
        Constants.Control.XBOX_SPIN_ROT_SENSITIVITY);
  }

  public Double getDirectionalSpeedLevel() {
    return speed_level;
  }

  public Double getSpinSpeedLevel() {
    return 3.14 * speed_level;
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
  public Boolean getTwerk() {
    return false; // drivebox.getRawButton(1);
  }
}
