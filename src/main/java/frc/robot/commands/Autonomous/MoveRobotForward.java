package frc.robot.commands.Autonomous;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveRobotForward extends Command {
    Drivetrain _drivetrain;
    public MoveRobotForward(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        _drivetrain.setVelocity(new ChassisSpeeds(-Constants.Autonomous.Score.FINAL_X_SPEED, 0, 0));
    }
}
