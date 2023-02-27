package frc.robot.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstants;

public class PhotonVisionCameraWrapper {

    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;

    public PhotonVisionCameraWrapper() {
        camera = new PhotonCamera(Constants.PhotonVisionConstants.CAMERA_NAME);

        photonPoseEstimator = new PhotonPoseEstimator(
            Constants.PhotonVisionConstants.APRIL_TAG_FIELD_LAYOUT,
            Constants.PhotonVisionConstants.POSE_STRATEGY, 
            camera, 
            Constants.PhotonVisionConstants.ROBOT_CENTER_TO_CAMERA
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
