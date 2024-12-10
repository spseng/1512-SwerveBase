package frc.robot.commands.TopLevel;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Arm _arm;

    public IntakeNote(Intake intake, Indexer indexer, Arm arm){
        _arm = arm;
        _indexer = indexer;
        _intake = intake;
        addRequirements(_indexer, _intake, _intake);

    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _arm.setAngle(Constants.Arm.ARM_INTAKE_ANGLE);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if (_arm.isLowestAngle()){
        _intake.setIntakeSpeed(Constants.In.INTAKE_SPEED);
        _indexer.setSpeed(Constants.In.INTAKE_SPEED);
        }else{
            
        }
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _intake.isInIntake();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        _intake.setIntakeSpeed(0);
        _indexer.setIndexerSpeedTimed(Constants.In.BACKOUT_TIME, Constants.In.BACKOUT_SPEED);
    }
    
}

