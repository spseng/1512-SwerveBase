package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetAmpAngle extends Command{
    private final Arm _arm;
    public SetAmpAngle(Arm arm){
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
        _arm.setAngle(Constants.Arm.AMP_SCORING_ANGLE);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (Constants.Arm.AMP_SCORING_ANGLE - Constants.Arm.TOLERANCE < _arm.getAngle() || Constants.Arm.AMP_SCORING_ANGLE + Constants.Arm.TOLERANCE > _arm.getAngle());
    }
    
}

