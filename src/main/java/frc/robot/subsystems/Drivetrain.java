package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends KillableSubsystem {

  // Creates differential kinematics
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH);

  // Create module objects
  private final DifferentialModule leftModule =
      new DifferentialModule(Constants.Differential.leftConstants);
  private final DifferentialModule rightModule =
      new DifferentialModule(Constants.Differential.rightConstants);

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(DriveCommandData driveCommandData) {
    // Data from driveCommandData
    boolean fieldRelative = driveCommandData.fieldRelative;
    double xSpeed = driveCommandData.xSpeed;
    double ySpeed = driveCommandData.ySpeed;
    double rot = driveCommandData.rot;

    // Calculates wheelSpeeds given optimal ChassisSpeeds given by control
    // scheme
    DifferentialDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(0))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Desaturates wheel speeds
    wheelSpeeds.desaturate(Constants.Differential.ROBOT_MAX_SPEED);

    // Sets state for each module
    leftModule.setDesiredState(wheelSpeeds.leftMetersPerSecond);
    rightModule.setDesiredState(wheelSpeeds.rightMetersPerSecond);
  }

  // set PID target to 0 but also immediately stop all modules
  @Override
  public void kill() {
    drive(new DriveCommandData(0, 0, 0, false));
    leftModule.stop();
    rightModule.stop();
  }

  @Override
  public void periodic() {
    leftModule.update();
    rightModule.update();
  }

  public void resetPose() {
    leftModule.resetDriveMotorPosition();
    rightModule.resetDriveMotorPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    // robot relative
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            leftModule.getModuleState(), rightModule.getModuleState()));
  }
}
