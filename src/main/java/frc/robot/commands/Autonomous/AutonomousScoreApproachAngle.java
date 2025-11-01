package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils.Vision.Camera;
import frc.robot.subsystems.Drivetrain;

public class AutonomousScoreApproachAngle extends Command {
    private final Drivetrain _drivetrain;
    private final Camera _visionProcessor;
    private final PIDController _rotationController;

    private static final double PID_MAX_OUTPUT = 1.0;  // Max motor output (e.g., 100% speed)
    private static final double PID_MIN_OUTPUT = -1.0;

    public AutonomousScoreApproachAngle(Drivetrain drivetrain, String cameraName) {
        _drivetrain = drivetrain;
        _visionProcessor = new Camera(cameraName, new Transform3d());
        _rotationController = new PIDController(Constants.Autonomous.Score.ROTATION_KP, Constants.Autonomous.Score.ROTATION_KI, Constants.Autonomous.Score.ROTATION_KD);

        _rotationController.setTolerance(Constants.Autonomous.Score.ROTATION_TOLERANCE);
        _rotationController.enableContinuousInput(-180, 180);
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        _rotationController.reset();
    }

    @Override
    public void execute() {
        double omega = -constraintOutput(_rotationController.calculate(_visionProcessor.getLargestTagTheta(), 180));
        //double Coefficient = Constants.Drivetrain.MAX_DRIVE_SPEED_MPS / Math.sqrt(vx * vx + vy * vy) * 0.3;
        double Coefficient = 20;
        omega *= 0.8;
        if(_visionProcessor.isTargetinSight() == false) {
            omega = 0;
        }
        SmartDashboard.putNumber("autonomous_error_x", _visionProcessor.getLargestTagX() - Constants.Autonomous.Score.DISTANCE_FACING_X);
        SmartDashboard.putNumber("autonomous_error_y", _visionProcessor.getLargestTagY());
        SmartDashboard.putNumber("autonomous_error_theta", _rotationController.getAccumulatedError());
        SmartDashboard.putNumber("autonomous_omega", omega);
        _drivetrain.setVelocity(new ChassisSpeeds(0, 0, omega));
    }

    @Override
    public boolean isFinished() {
        return ( _rotationController.atSetpoint()) || !_visionProcessor.isTargetinSight(); // This command never finishes on its own, it needs to be interrupted
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            _drivetrain.setVelocity(new ChassisSpeeds());  // Stop the robot when interrupted
        }
    }

    private double constraintOutput(double output) {
        if (output > PID_MAX_OUTPUT) return PID_MAX_OUTPUT;
        if (output < PID_MIN_OUTPUT) return PID_MIN_OUTPUT;
        return output;
    }
}