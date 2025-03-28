package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils.Vision.VisionProcessor;
import frc.robot.subsystems.Drivetrain;

public class AutonomousScoreApproach extends Command {
    private final Drivetrain _drivetrain;
    private final VisionProcessor _visionProcessor;

    private final PIDController _driveXController;
    private final PIDController _driveYController;
    private final PIDController _rotationController;

    private static final double PID_MAX_OUTPUT = 1.0;  // Max motor output (e.g., 100% speed)
    private static final double PID_MIN_OUTPUT = -1.0;

    public AutonomousScoreApproach(Drivetrain drivetrain, String cameraName) {
        _drivetrain = drivetrain;
        _visionProcessor = new VisionProcessor(cameraName);
        _driveXController = new PIDController(Constants.Autonomous.Score.DRIVE_X_KP, Constants.Autonomous.Score.DRIVE_X_KI, Constants.Autonomous.Score.DRIVE_X_KD);
        _driveYController = new PIDController(Constants.Autonomous.Score.DRIVE_Y_KP, Constants.Autonomous.Score.DRIVE_Y_KI, Constants.Autonomous.Score.DRIVE_Y_KD);
        _rotationController = new PIDController(Constants.Autonomous.Score.ROTATION_KP, Constants.Autonomous.Score.ROTATION_KI, Constants.Autonomous.Score.ROTATION_KD);
        _rotationController.enableContinuousInput(-180, 180);
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
        double vx = constraintOutput(_driveXController.calculate(_visionProcessor.getLargestTagX(), Constants.Autonomous.Score.DISTANCE_FACING_X));
        double vy = constraintOutput(_driveYController.calculate(_visionProcessor.getLargestTagY(), 0));
        double omega = constraintOutput(_rotationController.calculate(_visionProcessor.getLargestTagTheta(), 180));
        //double Coefficient = Constants.Drivetrain.MAX_DRIVE_SPEED_MPS / Math.sqrt(vx * vx + vy * vy) * 0.3;
        double Coefficient = 0.5;
        vx *= Coefficient;
        vy *= Coefficient;
        SmartDashboard.putNumber("autonomous_vx", vx);
        SmartDashboard.putNumber("autonomous_vy", vy);
        SmartDashboard.putNumber("autonomous_omega", omega);
        _drivetrain.setVelocity(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        return (_driveXController.atSetpoint() && _driveYController.atSetpoint() && _rotationController.atSetpoint()); // This command never finishes on its own, it needs to be interrupted
    }

    @Override
    public void end(boolean interrupted) {
        //_drivetrain.setVelocity(new ChassisSpeeds()); // Stop when finished
    }

    private double constraintOutput(double output) {
        if (output > PID_MAX_OUTPUT) return PID_MAX_OUTPUT;
        if (output < PID_MIN_OUTPUT) return PID_MIN_OUTPUT;
        return output;
    }
}