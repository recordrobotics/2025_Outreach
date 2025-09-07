package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.io.DifferentialModuleIO;
import frc.robot.utils.ModuleConstants;

public class DifferentialModuleSim implements DifferentialModuleIO, AutoCloseable {

  private final double periodicDt;

  private static double[] graph = new double[4];

  private final SparkMax m_driveMotor;
  private final SparkMaxSim m_driveMotorSim;

  private final SparkMax m_driveMotorFollower;
  private final SparkMaxSim m_driveMotorFollowerSim;

  private final DCMotor wheelMotor = DCMotor.getNEO(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              wheelMotor, 0.001, Constants.Differential.NEO_GEAR_RATIO),
          wheelMotor);
          private final DCMotorSim wheelSimModelFollower =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  wheelMotor, 0.001, Constants.Differential.NEO_GEAR_RATIO),
              wheelMotor);
  // private final AbstractDriveTrainSimulation drivetrainSim;

  private final double DRIVE_GEAR_RATIO;
  private final double WHEEL_DIAMETER;
  public double speedMetersPerSecond;

  public DifferentialModuleSim(double periodicDt, ModuleConstants m) {
    this.periodicDt = periodicDt;
    // this.drivetrainSim = drivetrainSim;

    m_driveMotor = new SparkMax(m.driveMotorChannel, MotorType.kBrushless);
    m_driveMotorFollower = new SparkMax(m.driveMotorFollowerChannel, MotorType.kBrushless);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(m_driveMotor).inverted(false);

    m_driveMotor.configure(
        followerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    m_driveMotorFollower.configure(
        followerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    m_driveMotorSim = new SparkMaxSim(m_driveMotor, wheelMotor);
    m_driveMotorFollowerSim = new SparkMaxSim(m_driveMotorFollower, wheelMotor);

    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;
    // if (coralDetectorSim != null)
    //   coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, true);
    // else coralDetectorSimValue = null;

    // if (coralDetectorSim != null) coralDetector.setSimDevice(coralDetectorSim);
    // else coralDetector.close();
  }

  private MedianFilter velocityFilter = new MedianFilter(10);

  @Override
  public void simulationPeriodic() {
    var voltage = m_driveMotorSim.getAppliedOutput() * m_driveMotorSim.getBusVoltage();

    wheelSimModel.setInputVoltage(voltage);
    wheelSimModel.update(periodicDt);

    m_driveMotorSim.iterate(
        Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
            * 60.0
            * DRIVE_GEAR_RATIO,
        RobotController.getBatteryVoltage(),
        periodicDt);


    var voltageFollower = m_driveMotorFollowerSim.getAppliedOutput() * m_driveMotorFollowerSim.getBusVoltage();

    wheelSimModelFollower.setInputVoltage(voltageFollower);
    wheelSimModelFollower.update(periodicDt);

    m_driveMotorFollowerSim.iterate(
        Units.radiansToRotations(wheelSimModelFollower.getAngularVelocityRadPerSec())
            * 60.0
            * DRIVE_GEAR_RATIO,
        RobotController.getBatteryVoltage(),
        periodicDt);
  }

  @Override
  public double getDriveWheelVelocity() {
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

  @Override
  public double getModuleState() {
    return getDriveWheelVelocity();
  }

  @Override
  public void resetDriveMotorPosition() {
    m_driveMotor.getEncoder().setPosition(0);
  }

  @Override
  public void setDesiredState(double speedMetersPerSecond) {
    this.speedMetersPerSecond = speedMetersPerSecond;
  }

  @Override
  public void stop() {
    m_driveMotor.close();
    m_driveMotorFollower.close();
  }

  @Override
  public void setupShuffleboard(double driveMotorChannel) {
    ShuffleboardUI.Test.addSlider("Drive " + driveMotorChannel, m_driveMotor.get(), -1, 1)
        .subscribe(m_driveMotor::set);
  }

  @Override
  public double getDriveWheelPosition() {
    double motorRots = m_driveMotor.getEncoder().getPosition();
    double wheelRots = motorRots / DRIVE_GEAR_RATIO;
    double wheelCircumference = WHEEL_DIAMETER * Math.PI;
    double wheelMeters = wheelRots * wheelCircumference;

    return wheelMeters;
  }

  @Override
  public void kill() {
    m_driveMotor.close();
    m_driveMotorFollower.close();
  }

  @Override
  public void update(
      double driveOutput, double speedMetersPerSecond, double driveFeedforwardOutput) {
    m_driveMotor.setVoltage(driveOutput + driveFeedforwardOutput);

    graph[m_driveMotor.getDeviceId() == 2 ? 0 : 2] = speedMetersPerSecond;

    SmartDashboard.putNumber("pos_" + m_driveMotor.getDeviceId(), getDriveWheelPosition());

    SmartDashboard.putNumberArray("drive", graph);
  }

  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_driveMotorFollower.close();
  }
}
