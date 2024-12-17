package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;

public class Autonomous {
    private final Drivetrain _drivetrain;
    private final SendableChooser<PathPlannerPath> _autoChooser;
    private final Map<String, PathPlannerPath> _preloadedPaths;

    public Autonomous(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
        _autoChooser = new SendableChooser<>();
        _preloadedPaths = new HashMap<>();

        RobotConfig robotConfig;
        try {
            robotConfig = RobotConfig.fromGUISettings();
            System.out.println("=================");
            System.out.println(robotConfig);
        } catch (Exception e) {
            e.printStackTrace();
            robotConfig = null;
        }

        if (robotConfig == null) {
            throw new RuntimeException("Failed to load robot configuration!");
        }
        AutoBuilder.configure(
                () -> _drivetrain.currentPoseSupplier().get(), // Explicit call
                pose -> _drivetrain.resetCurrentPoseConsumer().accept(pose), // Explicit call
                () -> _drivetrain.currentChassisSpeedsSupplier().get(), // Explicit call
                _drivetrain::setVelocity, // Same as before
                _drivetrain.getController(), // Assuming this works
                robotConfig, // Assuming this works
                () -> _drivetrain.isRedAllianceSupplier().getAsBoolean(), // Explicit call
                _drivetrain
        );

        preloadTrajectories();
        populateChoreoAutoChooser();
        SmartDashboard.putData("Auto Chooser", _autoChooser);
    }

    private void preloadTrajectories() {
        for (String pathName : Constants.Autonomous.PATH_PLANNER_PATHS) {
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                assert _preloadedPaths != null;
                _preloadedPaths.put(pathName, path);
                System.out.println("Successfully loaded path: " + pathName);
                SmartDashboard.putString(pathName,"YAY");
            } catch (Exception e) {
                System.err.println("Failed to load path: " + pathName);
                SmartDashboard.putString(pathName,"NOOO because " + e.getMessage());
                e.printStackTrace();
            }
        }
    }

    private void populateChoreoAutoChooser() {
        for (Map.Entry<String, PathPlannerPath> entry : _preloadedPaths.entrySet()) {
            String pathName = entry.getKey();
            PathPlannerPath path = entry.getValue();
            _autoChooser.addOption(pathName, path);
            SmartDashboard.putString("scanning: " + pathName, "going well");
        }

        if (!_preloadedPaths.isEmpty()) {
            String defaultPathName = _preloadedPaths.keySet().iterator().next();
            _autoChooser.setDefaultOption("Default: " + defaultPathName, _preloadedPaths.get(defaultPathName));
        } else {
            System.out.println("No PathPlanner paths preloaded!");
            SmartDashboard.putString("No PathPlanner paths preloaded!", "NOOO");
        }
    }

    public PathPlannerPath getSelectedPath() {
        return _autoChooser.getSelected();
    }
}
