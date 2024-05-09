package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class IdleIndexer extends Command {

    private final Indexer _indexer;
    public IdleIndexer(Indexer indexer){
        _indexer = indexer;
        addRequirements(_indexer);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _indexer.setSpeed(0);
    }
    
}
