package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {

    private final Drivetrain _drivetrain;
    private final OI _oi;
    private final int[] segmentationArray = new int[360 / 5];
    // creates 72 segments and 5 degrees each


    public Drive(OI oi, Drivetrain drivetrain) {
        _drivetrain = drivetrain;
        _oi = oi;
        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub

    }

    @SuppressWarnings("removal")
    @Override
    public void execute() {

        double vx;
        double vy;

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360.0 / segmentationArray.length;
            segmentationArray[i] = (int) (angle * i);
        }
        // creates segment of angles
        
        double rot = -_oi.getRotationX(); // radians per second
        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
                // gets x and y in a unit circle based on segmentation array.
                // basically normalising the x and y.

        vx = vec.x() * Constants.Drivetrain.MAX_DRIVE_SPEED_MPS; // mps
        vy = vec.y() * Constants.Drivetrain.MAX_DRIVE_SPEED_MPS; // mps

        rot = Math.signum(rot) * rot * rot; // square rot without loosing plus or minus
        rot = rot * Constants.Drivetrain.MAX_ANG_VEL;

        //SmartDashboard.putNumber("vx", vx);
        //SmartDashboard.putNumber("vy", vy);
        //SmartDashboard.putNumber("rot", rot);
        //SmartDashboard.putNumber("DESIRED_VELOCITY", Math.hypot(vx, vy));


        Rotation2d rotation = _drivetrain.isRedAlliance() ? _drivetrain.getHeading().plus(new Rotation2d(Math.PI)) : _drivetrain.getHeading();
        // determine the current rotation
        // switches 180 degree depending on the current alliance

        if (Math.abs(rot) > Constants.Drivetrain.ROTATION_DEADBAND || Math.abs(vx) > Constants.Drivetrain.TRANSLATION_DEADBAND || Math.abs(vy) > Constants.Drivetrain.TRANSLATION_DEADBAND) {
            _drivetrain.readModules();
            _drivetrain.setVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, rot, rotation));
            //_drivetrain.setVelocity(new ChassisSpeeds(vx, vy, rot));
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
