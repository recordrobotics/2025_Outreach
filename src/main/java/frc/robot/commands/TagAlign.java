package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.PVCamera.VisionReading;
import frc.robot.utils.DriveCommandData;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class TagAlign extends Command {

  private static final double ZERO_WIDTH = 0.025;
  private static final double MAX_ANGLE = 60.0;
  private static final double TARGET_OFFSET_YAW = 0.0;

  private Map<Integer, Double> lastShotAtTagTimes = new HashMap<>();
  private int currentTargetId = -1;

  private PIDController pid = new PIDController(1.6, 0, 0.2);

  public TagAlign() {
    addRequirements(RobotContainer._drivetrain);
  }

  @Override
  public void initialize() {
    currentTargetId = -1;
  }

  /**
   * Takes an input value between -1 and 1 and scales it to the proportion to which it's absolute
   * value is between a minimum threshold and 1 (Function returns 0 if input < threshold)
   *
   * @param input
   * @param threshold
   * @return
   */
  public static double interpolatedDeadZone(double input, double threshold) {
    if (threshold < 0 || threshold >= 1) {
      throw new IllegalArgumentException("Threshold must be between 0 and 1");
    }

    // How much the input is above the threshold (absolute value)
    double subtractThreshold = Math.max(0, Math.abs(input) - threshold);
    // What proportion (threshold to value) is of (threshold to 1)
    double proportion = subtractThreshold / (1 - threshold);
    // Brings back the correct sign and returns
    return Math.signum(input) * proportion;
  }

  @Override
  public void execute() {
    VisionReading currentTarget = getCurrentTargetReading();
    if (currentTarget == null) {
      currentTarget = getOldestVisibleTag();
    }

    driveTowardsTarget(currentTarget);

    Logger.recordOutput("TagAlign/CurrentTarget", currentTargetId);
  }

  public void declareShotAtCurrentTarget() {
    if (currentTargetId != -1) {
      lastShotAtTagTimes.put(currentTargetId, Timer.getTimestamp());
      currentTargetId = -1;
    }
  }

  private VisionReading getCurrentTargetReading() {
    if (currentTargetId == -1) return null;

    for (var reading : RobotContainer.pvCamera.getReadings().entrySet()) {
      if (reading.getValue().isVisible() && reading.getKey() == currentTargetId) {
        return reading.getValue();
      }
    }

    return null;
  }

  private VisionReading getOldestVisibleTag() {
    VisionReading target = null;
    double oldestTime = Double.MAX_VALUE;

    for (var reading : RobotContainer.pvCamera.getReadings().entrySet()) {
      if (reading.getValue().isVisible()) {
        double lastShotAt = lastShotAtTagTimes.getOrDefault(reading.getKey(), 0.0);
        if (lastShotAt < oldestTime) {
          oldestTime = lastShotAt;
          target = reading.getValue();
          currentTargetId = reading.getKey();
        }
      }
    }

    return target;
  }

  private void driveTowardsTarget(VisionReading target) {
    AbstractControl _controls = ShuffleboardUI.Overview.getControl();

    DriveCommandData drive = _controls.getDriveCommandData();

    if (target != null) {
      double yawError = target.getLastSeenAtYaw(TARGET_OFFSET_YAW) / MAX_ANGLE;
      yawError = interpolatedDeadZone(yawError, ZERO_WIDTH);
      double rotationSpeed = pid.calculate(yawError, 0.0);
      Logger.recordOutput("TagAlign/YawError", yawError);
      drive.rot = rotationSpeed;
    }

    RobotContainer._drivetrain.drive(drive);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
