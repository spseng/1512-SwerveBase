package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmAvoidElevator extends Command {
    
    private final Arm _arm;

    public ArmAvoidElevator(Arm arm){
        _arm = arm;
    }
    @Override
    public void initialize() {
        _arm.setArmPosition(Constants.Arm.AVOID_ELEVATOR_POSITION);
    }

    @Override
    public void execute() {
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _arm.isAtTarget();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
