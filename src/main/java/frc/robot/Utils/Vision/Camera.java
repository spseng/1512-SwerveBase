package frc.robot.Utils.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
    private final String _camName;
    private final double _camX;
    private final double _camY;
    private final double _camZ;
    private final double _camPitch;
    private final double _camYaw;
    private final double _camRoll;

    private final PhotonCamera _photonCamera;

    public Camera(String camName, double x, double y, double z, double pitch, double yaw, double roll) {
        _camName = camName;
        _camX = x;
        _camY = y;
        _camZ = z;
        _camPitch = pitch;
        _camYaw = yaw;
        _camRoll = roll;
        _photonCamera = new PhotonCamera(_camName);
    }

    public Transform3d getCameraLocation() {
        return new Transform3d(_camX, _camY, _camZ, new Rotation3d(_camPitch, _camYaw, _camRoll));
    }

    //public PhotonPipelineResult getLatestResult() { return _photonCamera.getLatestResult(); }

    public boolean isTargetinSight() {
        return _photonCamera.getLatestResult().hasTargets();
    }

}
