package frc.robot.subsystems.io.stub;

import frc.robot.subsystems.io.DifferentialModuleIO;

public class DifferentialModuleStub implements DifferentialModuleIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  public DifferentialModuleStub(double periodicDt) {
    this.periodicDt = periodicDt;
  }

  @Override
  public double getDriveWheelVelocity() {
    return 0;
  }

  @Override
  public double getModuleState() {
    return 0;
  }

  @Override
  public void resetDriveMotorPosition() {}

  @Override
  public void setDesiredState(double speedMetersPerSecond) {}

  @Override
  public void stop() {}

  @Override
  public void setupShuffleboard(double driveMotorChannel) {}

  @Override
  public void close() throws Exception {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public double getDriveWheelPosition() {
    return 0;
  }

  // @Override
  // public void update(double driveMotorChannel) {}

  @Override
  public void kill() {}

  @Override
  public void update(
      double driveOutput, double speedMetersPerSecond, double driveFeedforwardOutput) {}
}
