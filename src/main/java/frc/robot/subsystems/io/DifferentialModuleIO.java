package frc.robot.subsystems.io;

public interface DifferentialModuleIO extends AutoCloseable {

  public double getDriveWheelVelocity();

  public double getModuleState();

  public void resetDriveMotorPosition();

  public void setDesiredState(double speedMetersPerSecond);

  public void stop();

  public void setupShuffleboard(double driveMotorChannel);

  public void simulationPeriodic();

  public double getDriveWheelPosition();

  // public void update(double driveOutput);

  public void kill();

  public void update(
      double driveOutput, double speedMetersPerSecond, double driveFeedforwardOutput);
}
