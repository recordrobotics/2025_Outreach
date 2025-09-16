package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.io.real.DifferentialModuleReal;
import frc.robot.subsystems.io.sim.DifferentialModuleSim;
import frc.robot.subsystems.io.sim.NavSensorSim;
import frc.robot.subsystems.io.stub.NavSensorStub;
import frc.robot.utils.DriveCommandData;
import org.littletonrobotics.junction.AutoLogOutput;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends KillableSubsystem {

  public final NavSensor nav;
  // Creates differential kinematics
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH);

  DifferentialDrivePoseEstimator m_poseEstimator =
      new DifferentialDrivePoseEstimator(
          m_kinematics,
          new Rotation2d(),
          0.0,
          0.0,
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public static DifferentialModule m_left;
  public static DifferentialModule m_right;

  // Init drivetrain
  public Drivetrain() {
    nav =
        new NavSensor(
            Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                ? new NavSensorStub()
                : new NavSensorSim());

    m_left =
        new DifferentialModule(
            Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                ? new DifferentialModuleReal(0.02, Constants.Differential.leftConstants, true, true)
                : new DifferentialModuleSim(0.02, Constants.Differential.leftConstants),
            Constants.Differential.leftConstants,
            1.0,
            1.0);

    m_right =
        new DifferentialModule(
            Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                ? new DifferentialModuleReal(
                    0.02, Constants.Differential.rightConstants, true, true)
                : new DifferentialModuleSim(0.02, Constants.Differential.rightConstants),
            Constants.Differential.rightConstants,
            1.0,
            1.0);

    m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics,
            nav.getAdjustedAngle(),
            m_left.getDriveWheelPosition(),
            m_right.getDriveWheelPosition(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  }

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
    m_poseEstimator.update(
        nav.getAdjustedAngle(), m_left.getDriveWheelPosition(), m_right.getDriveWheelPosition());
  }

  @Override
  public void simulationPeriodic() {
    m_left.simulationPeriodic();
    m_right.simulationPeriodic();
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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
