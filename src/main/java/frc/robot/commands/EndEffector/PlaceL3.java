package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class PlaceL3 extends Command {

    private EndEffector _endEffector;
    private Arm _arm;
    
    public PlaceL3(EndEffector endEffector, Arm arm){
        _endEffector = endEffector;
        _arm = arm;


    }
    @Override
    public void initialize() {
    _arm.setArmPosition(Constants.Arm.L3_ANGLE);    
        
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _endEffector.setIntakeSpeed(Constants.In.EJECT_SPEED);
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
