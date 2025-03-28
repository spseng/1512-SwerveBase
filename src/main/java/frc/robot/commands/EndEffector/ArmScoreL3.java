package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmScoreL3 extends Command {
    
    private final Arm _arm;

    public ArmScoreL3(Arm arm){
        _arm = arm;
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        _arm.setArmPosition(Constants.Arm.L3_ANGLE);
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
