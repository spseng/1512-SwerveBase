package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonTest extends SubsystemBase {
    private final Pigeon2 _pigeon;
    public PigeonTest() {
        _pigeon = new Pigeon2(18);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pigeon_yaw", _pigeon.getYaw().getValueAsDouble());
        SmartDashboard.updateValues();
    }
}