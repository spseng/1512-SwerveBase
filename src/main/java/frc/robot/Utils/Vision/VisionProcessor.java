package frc.robot.Utils.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class VisionProcessor extends SubsystemBase {
    private final Camera _testCam;

    public VisionProcessor() {
        // Provide actual camera name and mount coordinates
        _testCam = new Camera("camera2", 0.2, 0.0, 0.5, 0.0, 0.0, 0.0);
    }

    public void updateDashboard() {
        if (!_testCam.isTargetinSight()) {
            SmartDashboard.putNumber("AprilTags Detected", 0); // No tags detected
            SmartDashboard.putString("AprilTag IDs", "None");
            return;
        }

        // Get the list of detected targets
        var targets = _testCam.getLatestResult().getTargets();
        int numTags = targets.size();

        // Extract tag IDs
        List<Integer> tagIDs = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            tagIDs.add(target.getFiducialId());
        }

        // Output to SmartDashboard
        SmartDashboard.putNumber("AprilTags Detected", numTags);
        SmartDashboard.putString("AprilTag IDs", tagIDs.toString());
    }

    @Override
    public void periodic() {
        this.updateDashboard();
    }
}