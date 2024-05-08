package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command{

    private final Drivetrain _drivetrain;
    private final OI _oi;
    private int segmentationArray[] = new int[360 / 5];
    

    public Drive(OI oi, Drivetrain drivetrain){
        _drivetrain = drivetrain;
        _oi = oi;
        addRequirements(_drivetrain);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        double vx = _oi.getDriveX(); // meters per second
        double vy = _oi.getDriveY(); // meters per second

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }

        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);

        vx = vec.x() * Constants.Drivetrain.DRIVE_SPEED; // mps
        vy = vec.y() * Constants.Drivetrain.DRIVE_SPEED; // mps

       
        double rot = -_oi.getRotationX(); // radians per second
        Rotation2d rotation = _drivetrain.isRedAlliance() ? _drivetrain.getHeading().plus(new Rotation2d(Math.PI)) : _drivetrain.getHeading();

        if (rot > Constants.Drivetrain.ROTATION_DEADBAND || vx > Constants.Drivetrain.TRANSLATION_DEADBAND || vy > Constants.Drivetrain.TRANSLATION_DEADBAND) {
           _drivetrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx,vy,rot, rotation));
        }  else {
            _drivetrain.setVelocity(new ChassisSpeeds());
        }

        // Check if either joystick is beyond the dead zone
       
    }
    
}
