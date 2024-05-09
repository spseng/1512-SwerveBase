package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Arm;

public class IdleArm extends Command {

    private final Arm _arm;
    private final OI _oi;

    public IdleArm(Arm arm, OI oi){
        _oi = oi;
        _arm = arm;
        addRequirements(_arm);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    if ( _oi.getArmUp() || !_arm.isHighestAngle()){
         _arm.setMotorOut(Constants.Arm.MANUAL_ARM_MOVE_SPEED);
    }
    if ( _oi.getArmDown() || !_arm.isLowestAngle()){
         _arm.setMotorOut(Constants.Arm.MANUAL_ARM_MOVE_SPEED);
    } else {
        _arm.setMotorOut(0);
    }


    }
    
}
