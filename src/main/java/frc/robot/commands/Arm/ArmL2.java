package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmL2 extends Command{

    private final Arm _arm;


    public ArmL2(Arm arm){

        _arm = arm;
       
    }

    @Override
    public void execute() {
     _arm.setArmPosition(Constants.Arm.L2_ANGLE);

    }
    
    
}
