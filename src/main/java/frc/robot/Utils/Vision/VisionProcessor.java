package frc.robot.Utils.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class VisionProcessor extends SubsystemBase {
    private final Camera _testCam;
    private final String _cameraName;

    public VisionProcessor(String cameraName) {
        _testCam = new Camera(cameraName, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0);
        _cameraName = cameraName;
    }

    public double getLargestTagX() {
        double[] pose = _testCam.getLargestTagPose();
        if (pose == null) return 0.0;
        return pose[0];
    }

    public double getLargestTagY() {
        double[] pose = _testCam.getLargestTagPose();
        if (pose == null) return 0.0;
        return pose[1];
    }

    public double getLargestTagTheta() {
        double[] pose = _testCam.getLargestTagPose();
        if (pose == null) return 0.0;
        return pose[2];
    }

    public void updateDashboard() {
        if (!_testCam.isTargetinSight()) {
            SmartDashboard.putNumber(_cameraName + ": AprilTags Detected", 0);
            SmartDashboard.putString(_cameraName + ": AprilTag IDs", "None");
            return;
        }

        var result = _testCam.getLatestResult();
        var targets = result.getTargets();
        int numTags = targets.size();

        List<Integer> tagIDs = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            tagIDs.add(target.getFiducialId());
        }

        double[] pose = _testCam.getLargestTagPose();
        SmartDashboard.putNumber(_cameraName + ": AprilTags Detected", numTags);
        SmartDashboard.putString(_cameraName + ": AprilTag IDs", tagIDs.toString());
        
        if (pose != null) {
            SmartDashboard.putNumber(_cameraName + ": Largest Tag X (m)", pose[0]);
            SmartDashboard.putNumber(_cameraName + ": Largest Tag Y (m)", pose[1]);
            SmartDashboard.putNumber(_cameraName + ": Largest Tag Theta (deg)", pose[2]);
        }
    }

    @Override
    public void periodic() {
        this.updateDashboard();
    }
}