package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Vision.Camera; // Assuming Camera is your custom Camera class
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final Camera camera1;
    private final Camera camera2;
    private Pose2d latestPose; // Store the latest valid pose
    private boolean hasValidPose; // Track validity of the latest pose
    private double lastPoseTimestamp; // Timestamp of the last valid pose

    public VisionSubsystem(Camera camera1, Camera camera2) {
        this.camera1 = camera1;
        this.camera2 = camera2;
        this.latestPose = new Pose2d(); // Initialize with a default pose
        this.hasValidPose = false; // Initially no valid pose
        this.lastPoseTimestamp = 0.0;
    }

    @Override
    public void periodic() {
        // Update pose estimation periodically
        updatePose();
        logToShuffleboard();
    }

    // Get raw pose from Camera 1 as a Transform3d
    public Optional<Transform3d> getPoseFromCamera1() {
        return camera1.getEstimatedPose();
    }

    // Get raw pose from Camera 2 as a Transform3d
    public Optional<Transform3d> getPoseFromCamera2() {
        return camera2.getEstimatedPose();
    }

    // Convert Transform3d to Pose2d (assuming field-relative 2D pose is needed)
    private Optional<Pose2d> transform3dToPose2d(Optional<Transform3d> transform) {
        if (transform.isPresent()) {
            Transform3d t = transform.get();
            // Extract 2D components (assuming z is ignored for a 2D field)
            return Optional.of(new Pose2d(
                t.getX(), 
                t.getY(), 
                new Rotation2d(t.getRotation().getZ()) // Assuming rotation around Z-axis
            ));
        }
        return Optional.empty();
    }

    // Determine the best pose from available cameras
    private void updatePose() {
        Optional<Transform3d> pose1 = getPoseFromCamera1();
        Optional<Transform3d> pose2 = getPoseFromCamera2();
        Optional<Pose2d> bestPose = Optional.empty();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Prioritize Camera 1 if available and valid
        if (pose1.isPresent()) {
            bestPose = transform3dToPose2d(pose1);
            if (bestPose.isPresent()) {
                latestPose = bestPose.get();
                hasValidPose = true;
                lastPoseTimestamp = currentTime;
                return;
            }
        }

        // Fallback to Camera 2 if Camera 1 is unavailable or invalid
        if (pose2.isPresent()) {
            bestPose = transform3dToPose2d(pose2);
            if (bestPose.isPresent()) {
                latestPose = bestPose.get();
                hasValidPose = true;
                lastPoseTimestamp = currentTime;
                return;
            }
        }

        // If no valid pose is found, mark as invalid but retain the last pose
        hasValidPose = false;
    }

    // Check if the latest pose is valid (e.g., recent and from a valid source)
    public boolean hasValidPose() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        // Consider the pose valid if it’s recent (e.g., within 0.1 seconds)
        boolean isRecent = (currentTime - lastPoseTimestamp) < 0.1;
        return hasValidPose && isRecent;
    }

    // Get the latest valid Pose2d for use in Drivetrain
    public Pose2d getLatestPose() {
        return latestPose;
    }

    // Log vision data to SmartDashboard for debugging
    private void logToShuffleboard() {
        SmartDashboard.putBoolean("Vision/HasValidPose", hasValidPose());
        SmartDashboard.putNumber("Vision/PoseX", latestPose.getX());
        SmartDashboard.putNumber("Vision/PoseY", latestPose.getY());
        SmartDashboard.putNumber("Vision/PoseRotation", latestPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/LastPoseTimestamp", lastPoseTimestamp);
    }
}