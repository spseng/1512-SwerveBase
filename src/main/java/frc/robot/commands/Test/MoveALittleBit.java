package frc.robot.commands.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class MoveALittleBit extends Command {
    private final Drivetrain _drivetrain;
    private double startTime;
    private static final double DURATION = 8.0;

    public MoveALittleBit(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
        //addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Rotation2d rotation = _drivetrain.getHeading();
        _drivetrain.setVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(3, 0, 0, rotation));
    }

    @Override
    public void end(boolean interrupted) {
        //_drivetrain.setVelocity(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= DURATION;
    }
}