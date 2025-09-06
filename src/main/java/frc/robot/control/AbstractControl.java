package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.DriveCommandData;

public abstract class AbstractControl {
  // Movement
  public abstract DriveCommandData getDriveCommandData();

  public abstract Boolean getShoot();

  public abstract Boolean getReverse();

  public abstract Boolean getTwerk();

  // Orient XY
  public static Pair<Double, Double> orientXY(Pair<Double, Double> input) {
    double inputX = input.getFirst();
    double inputY = input.getSecond();

    return switch (ShuffleboardUI.Overview.getDriverOrientation()) {
      case XAxisTowardsTrigger -> new Pair<Double, Double>(inputY, inputX);
      case YAxisTowardsTrigger -> new Pair<Double, Double>(-inputX, inputY);
      default -> new Pair<Double, Double>(0.0, 0.0);
    };
  }

  // Orient Angle
  public static Rotation2d orientAngle(Rotation2d angle) {
    return switch (ShuffleboardUI.Overview.getDriverOrientation()) {
      case XAxisTowardsTrigger -> new Rotation2d(angle.getRadians() + Math.PI / 2);
      case YAxisTowardsTrigger -> new Rotation2d(angle.getRadians() + Math.PI);
      default -> angle;
    };
  }
}
