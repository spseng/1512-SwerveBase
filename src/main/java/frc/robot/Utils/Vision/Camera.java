package frc.robot.Utils.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
    private final String _camName;
    private final PhotonCamera _photonCamera;
    private final Transform3d _cameraTransform;

    public Camera(String camName, double x, double y, double z, 
                 double pitch, double yaw, double roll) {
        _camName = camName;
        _photonCamera = new PhotonCamera(_camName);
        _cameraTransform = new Transform3d(
            x, y, z,
            new Rotation3d(pitch, yaw, roll)
        );
    }

    public Transform3d getCameraLocation() {
        return _cameraTransform;
    }

    public PhotonPipelineResult getLatestResult() { 
        return _photonCamera.getLatestResult(); 
    }

    public boolean isTargetinSight() {
        return getLatestResult().hasTargets();
    }
}