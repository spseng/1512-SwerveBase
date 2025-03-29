package frc.robot.commands.Score;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.Vector2d;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class EndEffectorOutakeConditional extends Command {
    private EndEffector _endEffector;



    public EndEffectorOutakeConditional( EndEffector endEffector){
      
        _endEffector = endEffector;
        addRequirements(_endEffector);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _endEffector.setIntakeSpeed(1);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        _endEffector.setIntakeSpeed(0);
    }
}
