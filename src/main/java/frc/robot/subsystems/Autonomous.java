package frc.robot.subsystems;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase {
    private static PathPlannerPath _path;
    public Autonomous(String trajectoryName) {
        _path = PathPlannerPath.fromChoreoTrajectory(trajectoryName);
    }
}
