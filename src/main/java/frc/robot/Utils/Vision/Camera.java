package frc.robot.Utils.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
    private final PhotonCamera _photonCamera;
    private final Transform3d _cameraTransform;

    public Camera(String camName, double x, double y, double z, 
                 double pitch, double yaw, double roll) {
        _photonCamera = new PhotonCamera(camName);
        _cameraTransform = new Transform3d(
            x, y, z,
            new Rotation3d(pitch, yaw, roll)
        );
    }

    public Transform3d getCameraLocation() {
        return _cameraTransform;
    }

    public PhotonPipelineResult getLatestResult() {
        var results = _photonCamera.getAllUnreadResults();
        return results.get(results.size() - 1);
    }

    public boolean isTargetinSight() {
        return getLatestResult().hasTargets();
    }

    public double[] getLargestTagPose() {
        if (!this.isTargetinSight()) return null;

        var result = this.getLatestResult();
        if (!result.hasTargets()) return null;

        PhotonTrackedTarget largestTarget = result.getBestTarget();
        if (largestTarget == null) return null;

        Transform3d cameraToTag = largestTarget.getBestCameraToTarget();

        double x = cameraToTag.getX();
        double y = cameraToTag.getY();

        // Convert rotation to Euler angles (proper yaw calculation)
        Rotation3d rotation = cameraToTag.getRotation();
        double theta = rotation.getZ() * (180.0 / Math.PI);

        return new double[]{x, y, theta};
    }
}