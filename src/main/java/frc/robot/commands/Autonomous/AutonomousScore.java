package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils.Vision.VisionProcessor;
import frc.robot.subsystems.Drivetrain;

public class AutonomousScore extends Command {
    private final Drivetrain _drivetrain;
    private final VisionProcessor _visionProcessor;

    private final PIDController _driveXController;
    private final PIDController _driveYController;
    private final PIDController _rotationController;

    public AutonomousScore(Drivetrain drivetrain, VisionProcessor visionProcessor, Constants.ReefDirection direction) {
        _drivetrain = drivetrain;
        _visionProcessor = visionProcessor;
        _driveXController = new PIDController(Constants.Autonomous.Score.DRIVE_X_KP, Constants.Autonomous.Score.DRIVE_X_KI, Constants.Autonomous.Score.DRIVE_X_KD);
        _driveYController = new PIDController(Constants.Autonomous.Score.DRIVE_Y_KP, Constants.Autonomous.Score.DRIVE_Y_KI, Constants.Autonomous.Score.DRIVE_Y_KD);
        _rotationController = new PIDController(Constants.Autonomous.Score.ROTATION_KP, Constants.Autonomous.Score.ROTATION_KI, Constants.Autonomous.Score.ROTATION_KD);
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        _driveXController.reset();
        _driveYController.reset();
        _rotationController.reset();
    }

    @Override
    public void execute() {
        double vx = _driveXController.calculate(_visionProcessor.getLargestTagX(), 0);
        double vy = _driveYController.calculate(_visionProcessor.getLargestTagY(), 0);
        double omega = _rotationController.calculate(_visionProcessor.getLargestTagTheta(), 0);
        _drivetrain.setVelocity(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        return false; // This command never finishes on its own, it needs to be interrupted
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.setVelocity(new ChassisSpeeds()); // Stop when finished
    }
}