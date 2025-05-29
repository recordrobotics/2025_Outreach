package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.subsystems.io.DifferentialModuleIO;
import frc.robot.subsystems.io.real.DifferentialModuleReal;
import frc.robot.utils.ModuleConstants;

public class DifferentialModule extends KillableSubsystem {
  public final DifferentialModuleIO io;
  public String side;

  public void setSide(String side) {
    this.side = side;
}

public String getSide() {
    return side;
}
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


    public static DifferentialModule create(DifferentialModuleIO io, String side) {
        DifferentialModule module = new DifferentialModule(io);
        module.setSide(side);
        return module;
    }
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and absolute turning encoder.
   *
   * @param m - a ModuleConstants object that contains all constants relevant for creating a swerve
   *     module. Look at ModuleConstants.java for what variables are contained
   */
  public DifferentialModule(DifferentialModuleIO io) {
    this.io = io;
    ModuleConstants m = side();

    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;

    // Sets up shuffleboard
    //  setupShuffleboard(m.driveMotorChannel);
  }

  ModuleConstants side() {
    if (side=="left") {
      return Constants.Differential.leftConstants;
    } else if (side=="right") {
      return Constants.Differential.rightConstants;
    } else {
      throw new IllegalArgumentException("Invalid side: " + side);
    }
  }

  /**
   * @return The current velocity of the drive motor (meters per second)
   */
  private double getDriveWheelVelocity() {
    return io.getDriveWheelVelocity();
  }

  private double getDriveWheelPosition() {
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
    io.update(driveOutput, driveFeedforwardOutput);
  }

  public void stop() {
    io.stop();
  }

  // SHUFFLEBOARD STUFF

  private void setupShuffleboard(double driveMotorChannel) {
    io.setupShuffleboard(driveMotorChannel);
  }

  @Override
  public void kill() {
    io.kill();
  }
}
