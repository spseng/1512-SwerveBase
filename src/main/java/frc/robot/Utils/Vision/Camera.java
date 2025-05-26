package frc.robot.Utils.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
    private final PhotonCamera _photonCamera;
    private final Transform3d _cameraTransform;

    public Camera(String camName, Transform3d cameraTransform) {
        _photonCamera = new PhotonCamera(camName);
        _cameraTransform = cameraTransform;
    }

    public Transform3d getCameraLocation() {
        return _cameraTransform;
    }

    public PhotonPipelineResult getLatestResult() {
        //var results = _photonCamera.getAllUnreadResults();
        //return results.get(results.size() - 1);
        return _photonCamera.getLatestResult();
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

    public double getLargestTagX() {
        double[] pose = getLargestTagPose();
        if (pose == null) return 0.0;
        return pose[0];
    }

    public double getLargestTagY() {
        double[] pose = getLargestTagPose();
        if (pose == null) return 0.0;
        return pose[1];
    }

    public double getLargestTagTheta() {
        double[] pose = getLargestTagPose();
        if (pose == null) return 0.0;
        return pose[2];
    }

    public Optional<Transform3d> getEstimatedPose() {
        var result = _photonCamera.getLatestResult();
        Optional<MultiTargetPNPResult> multiTagResult = result.getMultiTagResult();
        
        if (multiTagResult.isPresent()) {
            var pnpResult = multiTagResult.get();
            return Optional.of(pnpResult.estimatedPose.alt);
        }
        return Optional.empty();
    }
    
}