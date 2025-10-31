package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorCTR;

public class HalfDownIntake extends Command {
    private final Elevator _elevator;

    private final EndEffectorCTR _endEffector;
  
    private final Arm _arm;

    private boolean colision = false;
    private boolean done = false;

    public HalfDownIntake(Elevator elevator, Arm arm, EndEffectorCTR endEffector){

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
        done = false;
        _elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT);
        if (_elevator.getCurrentHeight() < 8){
            done = true;
        }
        // colision = _arm.isColision();
        // if (colision && (_elevator.getCurrentHeight() > 4)){
        //     _arm.setArmPosition(Constants.Arm.ARM_SAFE_ANGLE);
        //     // if ((Math.abs(_arm.getCurrentAngle() - Constants.Arm.ARM_SAFE_ANGLE)) > .05 ){
        //     //     done = true;
        //     // }
        // }else {
        //     done = true;
        // }
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return done;
    }
    
}
