package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command{

    private final Drivetrain _drivetrain;
    private final OI _oi;
    

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
        double leftX = _oi.getDriveX();
        double leftY = _oi.getDriveY();

        double leftR = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        double driveSpeed = (leftR < Constants.Drivetrain.TRANSLATION_DEADBAND) ? 0 : leftR * Constants.Drivetrain.DRIVE_SPEED;

        double rightX = -_oi.getRotationX();
       

        if (rightX > Constants.Drivetrain.ROTATION_DEADBAND || rightX < -Constants.Drivetrain.ROTATION_DEADBAND) {
            _drivetrain.rotate(-rightX);
        }
        else {
            _drivetrain.rotate(0);
        }

        // Check if either joystick is beyond the dead zone
        if (driveSpeed > 0) {
            _drivetrain.move(leftX, leftY);
        }  else {
            _drivetrain.move();
        }
    }
    
}
