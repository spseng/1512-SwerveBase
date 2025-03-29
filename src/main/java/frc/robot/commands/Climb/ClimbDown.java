package frc.robot.commands.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Climb;

public class ClimbDown extends Command {
    private Climb _climb;
    public ClimbDown(Climb climb) {
        _climb = climb;
        addRequirements(_climb);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _climb.setClimbSpeed(Constants.Climb.CLIMB_DOWN_SPEED);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        _climb.setClimbSpeed(0);
    }
}
