package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class IdleShooter extends Command {

    private final Shooter _shooter;
    public IdleShooter(Shooter shooter){
        _shooter = shooter;
        addRequirements(_shooter);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        //_shooter.setSpeed(Constants.Shooter.IDLE_SHOOTER_SPEED);
    _shooter.setSpeed(0);
    }
    
}

