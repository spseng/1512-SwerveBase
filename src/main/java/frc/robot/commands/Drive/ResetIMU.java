package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ResetIMU extends Command{
    
private Drivetrain _drivetrain;
private long _timeout;

    public ResetIMU(Drivetrain drivetrain){
        _drivetrain = drivetrain;
        addRequirements(_drivetrain);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _timeout = System.currentTimeMillis() + 100;
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _drivetrain.ZeroIMU();
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (System.currentTimeMillis() > _timeout);
    }
}
