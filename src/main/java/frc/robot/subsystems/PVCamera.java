package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PVCamera extends SubsystemBase {
  private int numTags = 0;
  private boolean hasVision = false;
  private boolean connected = false;

  private final PhotonCamera camera;

  private final VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;
  private static final SimCameraProperties SIM_CAMERA_PROPERTIES;

  private static final int DOWNSCALE = 2;
  private static final Pair<Integer, Integer> CAMERA_RESOLUTION = new Pair<>(3264, 2448);
  private static final Pair<Integer, Integer> ALGORITHM_RESOLUTION =
      new Pair<>(
          CAMERA_RESOLUTION.getFirst() / DOWNSCALE, CAMERA_RESOLUTION.getSecond() / DOWNSCALE);
  private static final double DIAGONAL_FOV = 150.0; // 180.0;
  private static final double FPS = 30.0;

  private static final VisionTargetSim[] targets = {
    new VisionTargetSim(new Pose3d(5, 0, 0, new Rotation3d(0, 0, 180)), TargetModel.kAprilTag36h11)
  };

  public class VisionReading {
    private Debouncer debouncer;
    public double lastSeenAtYawPixels;
    public boolean visibleThisFrame;

    private boolean visible = false;

    public VisionReading(double lastSeenAtYawPixels, boolean visibleThisFrame) {
      this.debouncer = new Debouncer(0.5, DebounceType.kBoth);
      this.lastSeenAtYawPixels = lastSeenAtYawPixels;
      this.visibleThisFrame = visibleThisFrame;
    }

    public void periodic() {
      visible = debouncer.calculate(visibleThisFrame);
    }

    public boolean isVisible() {
      return visible;
    }
  }

  private HashMap<Integer, VisionReading> readings = new HashMap<>();

  static {
    if (!isReal()) {
      SIM_CAMERA_PROPERTIES = new SimCameraProperties();
      SIM_CAMERA_PROPERTIES.setCalibration(
          ALGORITHM_RESOLUTION.getFirst(),
          ALGORITHM_RESOLUTION.getSecond(),
          Rotation2d.fromDegrees(DIAGONAL_FOV));
      SIM_CAMERA_PROPERTIES.setFPS(FPS);
    } else {
      SIM_CAMERA_PROPERTIES = null;
    }
  }

  public PVCamera(String name) {
    camera = new PhotonCamera(name);

    if (!isReal()) {
      visionSim = new VisionSystemSim("main");
      // visionSim.addVisionTargets(targets);
      visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
      cameraSim = new PhotonCameraSim(camera, SIM_CAMERA_PROPERTIES);
      cameraSim.enableDrawWireframe(true);
      visionSim.addCamera(cameraSim, Transform3d.kZero);
    } else {
      visionSim = null;
      cameraSim = null;
    }
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(RobotContainer._drivetrain.getPose());
  }

  public boolean isConnected() {
    return connected;
  }

  public boolean hasVision() {
    return hasVision;
  }

  public int getNumTags() {
    return numTags;
  }

  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  private static boolean isReal() {
    return Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL;
  }

  @Override
  public void periodic() {
    updateConnectionStatus();
    processFrame();
    logValues();
  }

  private void updateConnectionStatus() {
    connected = camera.isConnected();
  }

  private void processFrame() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      for (VisionReading r : readings.values()) {
        r.visibleThisFrame = false;
      }
    }

    for (PhotonPipelineResult result : results) {
      result
          .getTargets()
          .forEach(
              target -> {
                if (target.getFiducialId() == -1) return;
                if (readings.containsKey(target.getFiducialId())) {
                  readings.get(target.getFiducialId()).lastSeenAtYawPixels = target.getYaw();
                  readings.get(target.getFiducialId()).visibleThisFrame = true;
                } else {
                  readings.put(target.getFiducialId(), new VisionReading(target.getYaw(), true));
                }
              });
    }

    for (VisionReading r : readings.values()) {
      r.periodic();
    }
  }

  public void logValues() {
    Logger.recordOutput("NumTags", numTags);
    Logger.recordOutput("HasVision", hasVision);
    Logger.recordOutput("Connected", connected);

    for (var entry : readings.entrySet()) {
      Logger.recordOutput(
          "Tag" + entry.getKey() + "/LastSeenAtYawPixels", entry.getValue().lastSeenAtYawPixels);
      Logger.recordOutput("Tag" + entry.getKey() + "/Visible", entry.getValue().isVisible());
    }
  }

  public Map<Integer, VisionReading> getReadings() {
    return readings;
  }
}
