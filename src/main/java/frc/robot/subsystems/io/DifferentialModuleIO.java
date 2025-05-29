package frc.robot.subsystems.io;

public interface DifferentialModuleIO extends AutoCloseable {
  public String side = "";
  public String getSide();

  public void setSide(String side);

  public double getDriveWheelVelocity();

  public double getModuleState();

  public void resetDriveMotorPosition();

  public void setDesiredState(double speedMetersPerSecond);

  public void stop();

  public void setupShuffleboard(double driveMotorChannel);

  public void simulationPeriodic();

  public double getDriveWheelPosition();

  public void update(double driveOutput);

  public void kill();

  public void update(double driveOutput, double driveFeedforwardOutput);
}
