 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class SimpleIntake extends Command {
    private final Intake _intake;
    private final Indexer _indexer;

    public SimpleIntake(Intake intake, Indexer indexer){
        
        _indexer = indexer;
        _intake = intake;
        addRequirements(_indexer, _intake);

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
        
        _intake.setIntakeSpeed(Constants.In.INTAKE_SPEED);
        _indexer.setSpeed(Constants.In.INTAKE_SPEED);
     
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
