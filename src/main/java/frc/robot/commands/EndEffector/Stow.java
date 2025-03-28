package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class Stow extends Command{

    private final Elevator _elevator;
    private final Arm _arm;
    private final EndEffector _endEffector;

    public Stow(Elevator elevator, Arm arm, EndEffector endEffector){

        _elevator = elevator;
        _arm = arm;
        _endEffector = endEffector;
    }
    @Override
    public void initialize() {
        
        
    }
    @Override
    public void execute() {
        _elevator.setTargetHeight(Constants.Elevator.STOW_HEIGHT);
        _arm.setArmPosition(Constants.Arm.STOW_POSITION);
        _endEffector.setIntakeSpeed(0);
    }

    
}
