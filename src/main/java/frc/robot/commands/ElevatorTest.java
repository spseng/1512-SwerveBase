package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ElevatorTest extends Command {

    private Elevator _elevator;

    private final OI _oi;

    private final int[] segmentationArray = new int[360 / 5];

    public ElevatorTest(OI oi, Elevator elevator){
        _oi = oi;
        _elevator = elevator;
        addRequirements(_elevator);
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
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

        double target_height = _elevator.getCurrentHeight() + vx;

        _elevator.setTargetHeight(_elevator.getCurrentHeight() + vx * 2);

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
