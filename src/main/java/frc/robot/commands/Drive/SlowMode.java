package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.subsystems.Drivetrain;

public class SlowMode extends Command {

    private Drivetrain _drivetrain;
    
    public SlowMode(Drivetrain drivetrain, KinematicLimits oldKinematicLimits){
        _drivetrain = drivetrain;
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
        _drivetrain.setKinematicLimits(Constants.Drivetrain.SLOW_MODE_KINEMATIC_LIMITS);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        _drivetrain.setKinematicLimits(Constants.Drivetrain.DRIVE_KINEMATIC_LIMITS);
    }

}
