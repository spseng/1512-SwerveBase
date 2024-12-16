package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ClimbUp extends Command{
    private Arm _arm;
    public ClimbUp(Arm arm){
        _arm = arm;
        addRequirements(_arm);

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
        _arm.setAngle(Constants.Arm.ARM_CLIMB_ANGLE);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (Constants.Arm.ARM_MAX_ANGLE - Constants.Arm.TOLERANCE < _arm.getAngle() || Constants.Arm.ARM_MAX_ANGLE + Constants.Arm.TOLERANCE > _arm.getAngle());
    }
    
}

