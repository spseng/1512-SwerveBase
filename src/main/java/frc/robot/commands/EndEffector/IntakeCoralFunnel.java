package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class IntakeCoralFunnel extends Command {
    private final EndEffector _endEffector;
    private final Arm _arm;
    private final Elevator _elevator;

    public IntakeCoralFunnel(EndEffector endEffector, Arm arm, Elevator elevator){

        _endEffector = endEffector;
        _arm = arm;
        _elevator = elevator;
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
        _elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT);
        _arm.setArmPosition(Constants.Arm.ARM_INTAKE_ANGLE);
        _endEffector.setIntakeSpeed(Constants.EndEffector.INTAKE_SPEED);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _endEffector.isCoralInIntake();
    }
    
    
    
}
