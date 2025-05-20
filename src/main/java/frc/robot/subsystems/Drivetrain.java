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
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH);

  // Create module objects
  private final DifferentialModule m_left =
      new DifferentialModule(Constants.Differential.leftConstants);
  private final DifferentialModule m_right =
      new DifferentialModule(Constants.Differential.rightConstants);

  // Init drivetrain
  public Drivetrain() {}

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
        m_kinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(0))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Desaturates wheel speeds
    wheelSpeeds.desaturate(Constants.Differential.robotMaxSpeed);

    // Sets state for each module
    m_left.setDesiredState(wheelSpeeds.leftMetersPerSecond);
    m_right.setDesiredState(wheelSpeeds.rightMetersPerSecond);
  }

  // set PID target to 0 but also immediately stop all modules
  @Override
  public void kill() {
    drive(new DriveCommandData(0, 0, 0, false));
    m_left.stop();
    m_right.stop();
  }

  @Override
  public void periodic() {
    m_left.update();
    m_right.update();
  }

  /** Resets the field relative position of the robot (mostly for testing). */
  public void resetStartingPose() {
    m_left.resetDriveMotorPosition();
    m_right.resetDriveMotorPosition();
  }

  /** Resets the pose to FrontSpeakerClose (shooter facing towards speaker) */
  public void resetDriverPose() {
    m_left.resetDriveMotorPosition();
    m_right.resetDriveMotorPosition();
  }

  /** Returns the current robot relative chassis speeds of the swerve kinematics */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(m_left.getModuleState(), m_right.getModuleState()));
  }
}
