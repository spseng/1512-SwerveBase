package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgaeReef extends Command {

    private Arm _arm;
    private EndEffector _endEffector;

    public IntakeAlgaeReef(Arm arm, EndEffector endEffector){
        _arm = arm;
        _endEffector = endEffector;
    }
    @Override
    public void initialize() {
        _arm.setArmPosition(Constants.Arm.ALGAE_POSITION);
        
        
    }
    @Override
    public void execute() {
        
        
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
