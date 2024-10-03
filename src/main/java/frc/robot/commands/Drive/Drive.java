package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Command {

    private final Drivetrain _drivetrain;
    private final OI _oi;
    private int segmentationArray[] = new int[360 / 5];


    public Drive(OI oi, Drivetrain drivetrain) {
        _drivetrain = drivetrain;
        _oi = oi;
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub

    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub

        double vx;
        double vy;

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }
        double rot = -_oi.getRotationX(); // radians per second
        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        SmartDashboard.putNumber("vec.x", vec.x());
        SmartDashboard.putNumber("vec.y", vec.y());
        SmartDashboard.putNumber("vec.x+vec.y", vec.x()+vec.y());

        vx = vec.x() * Constants.Drivetrain.MAX_DRIVE_SPEED_MPS; // mps
        vy = vec.y() * Constants.Drivetrain.MAX_DRIVE_SPEED_MPS; // mps

        rot = Math.signum(rot) * rot * rot;
        rot = rot * Constants.Drivetrain.MAX_ANG_VEL;


        Rotation2d rotation = _drivetrain.isRedAlliance() ? _drivetrain.getHeading().plus(new Rotation2d(Math.PI)) : _drivetrain.getHeading();

        if (rot > Constants.Drivetrain.ROTATION_DEADBAND || vx > Constants.Drivetrain.TRANSLATION_DEADBAND || vy > Constants.Drivetrain.TRANSLATION_DEADBAND) {
            _drivetrain.readModules();
            _drivetrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, rotation));
        } else {
            _drivetrain.readModules();
            _drivetrain.setVelocity(new ChassisSpeeds());
        }

        // Check if either joystick is beyond the dead zone

    }

    @Override
    public boolean isFinished() {
        return false; // This command never finishes on its own, it needs to be interrupted
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.setVelocity(new ChassisSpeeds()); // Stop the drivetrain when the command ends
    }

}
