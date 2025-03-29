package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Vision.Camera;
import frc.robot.Utils.Vision.VisionProcessor;

public class VisionSubsystem extends SubsystemBase {
    private final VisionProcessor camera1;
    private final VisionProcessor camera2;

    public VisionSubsystem(String camera1Name, String camera2Name) {
        camera1 = new VisionProcessor(camera1Name);
        camera2 = new VisionProcessor(camera2Name);
    }

    public VisionProcessor getCamera1() {
        return camera1;
    }

    public VisionProcessor getCamera2() {
        return camera2;
    }

    @Override
    public void periodic() {
        camera1.updateDashboard();
        camera2.updateDashboard();
    }
}
