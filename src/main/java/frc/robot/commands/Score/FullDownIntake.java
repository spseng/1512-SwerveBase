package frc.robot.commands.Score;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorCTR;

public class FullDownIntake extends Command {
    private final Elevator _elevator;

    private final EndEffectorCTR _endEffector;
  
    private final Arm _arm;

   

    public FullDownIntake(Elevator elevator, Arm arm, EndEffectorCTR endEffector){

        _elevator = elevator;
        _arm = arm;
        _endEffector = endEffector;

    }
    @Override
    public void initialize() {
        
       
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT);
        _arm.setArmPosition(Constants.Arm.ARM_INTAKE_ANGLE);
        _endEffector.setIntakeSpeed(Constants.EndEffector.INTAKE_SPEED);
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    
}
