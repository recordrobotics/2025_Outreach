package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.Constants;
import frc.robot.subsystems.io.DifferentialModuleIO;
import frc.robot.utils.ModuleConstants;

public class DifferentialModule{
  public final DifferentialModuleIO io;

  private static double[] graph = new double[4];

  // // Creates variables for motors and absolute encoders
  // private final SparkMax m_driveMotor;
  // private final SparkMax m_driveMotorFollower;

  private final double DRIVE_GEAR_RATIO;
  private final double WHEEL_DIAMETER;
  public double speedMetersPerSecond;

  // Creates PID Controllers
  private final PIDController drivePIDController =
      new PIDController(
          Constants.Differential.NEO_KP,
          Constants.Differential.NEO_KI,
          Constants.Differential.NEO_KD);

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          Constants.Differential.NEO_FEEDFORWARD_KS, Constants.Differential.NEO_FEEDFORWARD_KV);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and absolute turning encoder.
   *
   * @param m - a ModuleConstants object that contains all constants relevant for creating a swerve
   *     module. Look at ModuleConstants.java for what variables are contained
   */
  public DifferentialModule(DifferentialModuleIO io, ModuleConstants m) {
    this.io = io;

    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;

    // Sets up shuffleboard
    //  setupShuffleboard(m.driveMotorChannel);
  }

  /**
   * @return The current velocity of the drive motor (meters per second)
   */
  public double getDriveWheelVelocity() {
    return io.getDriveWheelVelocity();
  }

  public double getDriveWheelPosition() {
    return io.getDriveWheelPosition();
  }

  /**
   * *custom function
   *
   * @return The current state of the module.
   */
  public double getModuleState() {
    return getDriveWheelVelocity();
  }

  /** resets drive motor position */
  public void resetDriveMotorPosition() {
    io.resetDriveMotorPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(double speedMetersPerSecond) {
    // Calculate the drive output from the drive PID controller then set drive
    // motor.
    this.speedMetersPerSecond = speedMetersPerSecond;
  }

  public void update() {
    double driveOutput =
        drivePIDController.calculate(getDriveWheelVelocity(), speedMetersPerSecond);
    double driveFeedforwardOutput = driveFeedForward.calculate(speedMetersPerSecond);
    io.update(driveOutput + driveFeedforwardOutput, speedMetersPerSecond, driveFeedforwardOutput);
  }

  public void stop() {
    io.stop();
  }

  // SHUFFLEBOARD STUFF

  private void setupShuffleboard(double driveMotorChannel) {
    io.setupShuffleboard(driveMotorChannel);
  }


  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}
