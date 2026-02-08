package frc.robot.subsystems.swervedrive;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase{
  public PhotonCamera camera;
  private AprilTagFieldLayout fieldLayout;
  private boolean hasFieldLayout = false;

  private VisionSystemSim visionSim = new VisionSystemSim("Limelight");
  private SimCameraProperties cameraSimProperties = new SimCameraProperties();
  private PhotonCameraSim cameraSim;
  private SwerveSubsystem swerve;

  public Vision(PhotonCamera camera, SwerveSubsystem swerve) {
    this.camera = camera;
    this.swerve = swerve;
    
    try {
      this.fieldLayout = Robot.isReal() ? new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/vision/2025-test-field.json") : AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    } catch (Exception exception) {
      System.err.println("ERROR: April tag layout file not found!");
      exception.printStackTrace();
    }

    if (Robot.isSimulation()) {
      this.visionSim.addAprilTags(this.fieldLayout);
      this.cameraSimProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
      this.cameraSimProperties.setCalibError(0.25, 0.08);
      this.cameraSimProperties.setFPS(20);
      this.cameraSimProperties.setAvgLatencyMs(35);
      this.cameraSimProperties.setLatencyStdDevMs(5);

      final Transform3d robotToCamera = new Transform3d(Constants.ROBOT_TO_CAMERA_POSE.getTranslation(), Constants.ROBOT_TO_CAMERA_POSE.getRotation());
      this.cameraSim = new PhotonCameraSim(camera, this.cameraSimProperties);
      this.visionSim.addCamera(this.cameraSim, robotToCamera);

    //local host configurations

    // cameraSim.enableRawStream(true);
    // cameraSim.enableProcessedStream(true);
    // cameraSim.enableDrawWireframe(true);
    }
  }

  // Robot relative Pose
  public Optional<Transform3d> getTagPose(int id) {
    Optional<PhotonPipelineResult> result = getLastResult();
    if (result.isEmpty()) {
      return Optional.empty();
    }

    List<PhotonTrackedTarget> targets = result.get().getTargets();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == id) {
        return Optional.of(target.getBestCameraToTarget());
      }
    }

    return Optional.empty();
  }

  public Optional<Pose2d> getFieldPose() {
    Optional<PhotonPipelineResult> result = getLastResult();
    if (result.isEmpty()) {
      return Optional.empty();
    }

    Optional<MultiTargetPNPResult> multiTagResult = result.get().getMultiTagResult();
    SmartDashboard.putBoolean("Has Mutitag Result", hasFieldLayout);
    if (multiTagResult.isPresent()) {
      hasFieldLayout = true;

      MultiTargetPNPResult fieldResult = multiTagResult.get();
      Transform3d robotTransform = fieldResult.estimatedPose.best;

      return Optional.of(new Pose2d(robotTransform.getX(), robotTransform.getY(), robotTransform.getRotation().toRotation2d()));
    }

    return Optional.empty();
  }

  public Optional<PhotonPipelineResult> getLastResult() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    return results.size() > 0 ? Optional.of(results.get(results.size() - 1)) : Optional.empty();
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public void updateSimulation() {
    visionSim.update(swerve.getPose());
    Field2d visionSimField = visionSim.getDebugField(); 
    SmartDashboard.putData("Vision Sim Field", visionSimField);
  }
}

