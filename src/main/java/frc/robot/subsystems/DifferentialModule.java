package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.io.DifferentialModuleIO;
import frc.robot.utils.ModuleConstants;
import frc.robot.Constants;

public class DifferentialModule extends KillableSubsystem {
  public final DifferentialModuleIO io;

  private static double[] graph = new double[4];

  // Creates variables for motors and absolute encoders
  private final SparkMax m_driveMotor;
  private final SparkMax m_driveMotorFollower;

  private final double DRIVE_GEAR_RATIO;
  private final double WHEEL_DIAMETER;
  public double speedMetersPerSecond;


      // Creates PID Controllers
      private final PIDController drivePIDController =
      new PIDController(
          Constants.Differential.NEO_KP,
          Constants.Differential.NEO_KI,
          Constants.Differential.NEO_KD,
          new TrapezoidProfile.Constraints(
              Constants.Differential.DriveMaxAngularVelocity, Constants.Differential.DriveMaxAngularAcceleration));

      private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(Constants.Differential.KRAKEN_DRIVE_FEEDFORWARD_KS, Constants.Differential.KRAKEN_DRIVE_FEEDFORWARD_KV);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and absolute turning encoder.
   *
   * @param m - a ModuleConstants object that contains all constants relevant for creating a swerve
   *     module. Look at ModuleConstants.java for what variables are contained
   */
  public DifferentialModule(DifferentialModuleIO io) {

    // Creates TalonFX objects
    m_driveMotor = new SparkMax(m.driveMotorChannel, MotorType.kBrushless);
    m_driveMotorFollower = new SparkMax(m.driveMotorFollowerChannel, MotorType.kBrushless);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(m_driveMotor).inverted(false);

    m_driveMotor.configure(
        new SparkMaxConfig().inverted(m.inverted),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_driveMotorFollower.configure(
        followerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Creates other variables
    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;

    // Sets motor speeds to 0
    m_driveMotor.set(0);


    // Sets up shuffleboard
    //  setupShuffleboard(m.driveMotorChannel);
  }

  /**
   * @return The current velocity of the drive motor (meters per second)
   */
  private double getDriveWheelVelocity() {
    // Convert motor velocity from RPM to rotations per second (RPS)
    double motorRPM = m_driveMotor.getEncoder().getVelocity(); // Use motor encoder to get RPM
    double motorRPS = motorRPM / 60.0; // divide by 60 seconds per minute to get RPS

    // Convert motor rotations per second to wheel meters per second
    double wheelCircumference = WHEEL_DIAMETER * Math.PI;
    double wheelRPS = motorRPS / DRIVE_GEAR_RATIO; // compensate for the drive gear ratio
    double wheelMetersPerSecond = wheelRPS * wheelCircumference;

    // Update graph with the current wheel velocity
    int graphIndex = (m_driveMotor.getDeviceId() == 2) ? 1 : 3;
    graph[graphIndex] = wheelMetersPerSecond;

    return wheelMetersPerSecond;
  }

  private double getDriveWheelPosition() {
    double motorRots = m_driveMotor.getEncoder().getPosition();
    double wheelRots = motorRots / DRIVE_GEAR_RATIO;
    double wheelCircumference = WHEEL_DIAMETER * Math.PI;
    double wheelMeters = wheelRots * wheelCircumference;

    return wheelMeters;
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
    m_driveMotor.getEncoder().setPosition(0);
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
    m_driveMotor.setVoltage(driveOutput + driveFeedforwardOutput);

    graph[m_driveMotor.getDeviceId() == 2 ? 0 : 2] = speedMetersPerSecond;

    SmartDashboard.putNumber("pos_" + m_driveMotor.getDeviceId(), getDriveWheelPosition());

    SmartDashboard.putNumberArray("drive", graph);
  }

  public void stop() {
    m_driveMotor.setVoltage(0);
  }

  // SHUFFLEBOARD STUFF

  private void setupShuffleboard(double driveMotorChannel) {
    ShuffleboardUI.Test.addSlider("Drive " + driveMotorChannel, m_driveMotor.get(), -1, 1)
        .subscribe(m_driveMotor::set);
  }

  @Override
  public void kill() {
    m_driveMotor.setVoltage(0);
  }
}
