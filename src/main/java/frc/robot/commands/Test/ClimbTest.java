package frc.robot.commands.Test;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.OI;

import frc.robot.subsystems.Climb;

public class ClimbTest extends Command {

    private Climb _climb;

    private final OI _oi;

    private final int[] segmentationArray = new int[360 / 5];

    public ClimbTest(OI oi, Climb climb){
        _oi = oi;
        _climb = climb;
        addRequirements(_climb);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /*
        double vx;
        double vy;

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360.0 / segmentationArray.length;
            segmentationArray[i] = (int) (angle * i);
        }
        // creates segment of angles
        
        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
                // gets x and y in a unit circle based on segmentation array.
                // basically normalising the x and y.

        vx = vec.x();
        vy = vec.y();

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        */

        double v = _oi.getElevatorY();
        /*/
        if(v == 0) {
            v = 0.0;
        }else {
            v -= 0.5;
        }
        */

        _climb.setClimbSpeed(v);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
   

    
}