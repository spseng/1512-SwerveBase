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

    public VisionProcessor() {
        _testCam = new Camera("camera2", 0.2, 0.0, 0.5, 0.0, 0.0, 0.0);
    }

    public void updateDashboard() {
        if (!_testCam.isTargetinSight()) {
            SmartDashboard.putNumber("AprilTags Detected", 0);
            SmartDashboard.putString("AprilTag IDs", "None");
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
        SmartDashboard.putNumber("AprilTags Detected", numTags);
        SmartDashboard.putString("AprilTag IDs", tagIDs.toString());
        
        if (pose != null) {
            SmartDashboard.putNumber("Largest Tag X (m)", pose[0]);
            SmartDashboard.putNumber("Largest Tag Y (m)", pose[1]);
            SmartDashboard.putNumber("Largest Tag Theta (deg)", pose[2]);
        }
    }

    @Override
    public void periodic() {
        this.updateDashboard();
    }
}