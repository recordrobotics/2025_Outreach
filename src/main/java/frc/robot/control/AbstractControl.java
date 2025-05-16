package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.DriveCommandData;

public abstract class AbstractControl {
  // Movement
  public abstract DriveCommandData getDriveCommandData();

  // Misc
  // public abstract Boolean getPoseReset();

  // public abstract Boolean getKillAuto();

  public abstract Boolean getShoot();

  public abstract Boolean getReverse();

  public abstract Boolean getTwerk();

  // Orient XY
  public static Pair<Double, Double> OrientXY(Pair<Double, Double> input) {
    double inputX = input.getFirst();
    double inputY = input.getSecond();

    switch (ShuffleboardUI.Overview.getDriverOrientation()) {
      case XAxisTowardsTrigger:
        return new Pair<Double, Double>(inputY, inputX);
      case YAxisTowardsTrigger:
        return new Pair<Double, Double>(-inputX, inputY);
      default:
        return new Pair<Double, Double>(0.0, 0.0);
    }
  }

  // Orient Angle
  public static Rotation2d OrientAngle(Rotation2d angle) {
    switch (ShuffleboardUI.Overview.getDriverOrientation()) {
      case XAxisTowardsTrigger:
        return new Rotation2d(angle.getRadians() + Math.PI / 2);
      case YAxisTowardsTrigger:
        return new Rotation2d(angle.getRadians() + Math.PI);
      default:
        return angle;
    }
  }
}
