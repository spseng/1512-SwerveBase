package frc.robot.commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoreL4 extends Command {

    private final Elevator _elevator;
  
    private final Arm _arm;

    private boolean colision = false;

    public ScoreL4(Elevator elevator, Arm arm){

        _elevator = elevator;
        _arm = arm;

    }
    @Override
    public void initialize() {

        colision = _arm.isColision();
       
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if (colision){
            _arm.setArmPosition(Constants.Arm.ARM_SAFE_ANGLE);
        } else {
            _elevator.setTargetHeight(Constants.Elevator.L4_HEIGHT);
            _arm.setArmPosition(Constants.Arm.L4_ANGLE);
        }
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    
}
