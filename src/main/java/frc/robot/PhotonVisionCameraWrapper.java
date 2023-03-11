package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonVisionCameraWrapper {
  public final static String CAMERA_NAME = "photonvision";

  // TODO: Enter actual values (In meters and degrees)
  public static final Transform3d ROBOT_CENTER_TO_CAMERA = new Transform3d(
      new Translation3d(0.0, 0.0, 0.0),
      new Rotation3d(Math.toRadians(180), 0, Math.toRadians(180)));

  public final static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

  static {
    try {
      APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      throw new RuntimeException("I/O exception with april tag field layout", e);
    }
  }
  // TODO: Choose Pose Strategy
  public final static PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP;

  PhotonCamera camera;
  PhotonPoseEstimator photonPoseEstimator;

  public PhotonVisionCameraWrapper() {
    camera = new PhotonCamera(CAMERA_NAME);

    photonPoseEstimator = new PhotonPoseEstimator(
        APRIL_TAG_FIELD_LAYOUT,
        POSE_STRATEGY,
        camera,
        ROBOT_CENTER_TO_CAMERA);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
