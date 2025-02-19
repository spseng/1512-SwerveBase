package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class PlaceL4 extends Command {

    private final EndEffector _endEffector;
    private final Arm _arm;
    private final Elevator _elevator;
    
    public PlaceL4(EndEffector endEffector, Arm arm, Elevator elevator){
        _endEffector = endEffector;
        _arm = arm;
        _elevator = elevator;


    }
    @Override
    public void initialize() {
    _arm.setArmPosition(Constants.Arm.L4_ANGLE); 
    _elevator.setTargetHeight(Constants.Elevator.L4_HEIGHT); 
        
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if (_elevator.isAtTarget() && _arm.isAtTarget()){_endEffector.setIntakeSpeed(Constants.EndEffector.PLACE_SPEED);
        }

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
