package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

public class PhotonVisionCameraWrapper {

    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;

    public PhotonVisionCameraWrapper() {
        camera = new PhotonCamera(Constants.PhotonVisionConstants.cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
            Constants.PhotonVisionConstants.aprilTagFieldLayout,
            Constants.PhotonVisionConstants.poseStrategy, 
            camera, 
            Constants.PhotonVisionConstants.robotCenterToCamera
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
