package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class IncrementUp extends Command{
private Arm _arm;
private boolean isFinished = false;

    public IncrementUp(Arm arm){
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
        if (!_arm.isHighestAngle()){
        _arm.setMotorDirect(0.4);
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
    //    _arm.setMotorDirect(0);
   }
}


