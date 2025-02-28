package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.AxisButton;
import frc.robot.Utils.Gamepad;
import frc.robot.Utils.Helpers;
import frc.robot.commands.Drive.ResetIMU;
import frc.robot.commands.Drive.SlowMode;
import frc.robot.commands.Drive.Snap;
import frc.robot.subsystems.Drivetrain;



public class OI {
    private Gamepad _driverGamepad;
    private Gamepad _operatorGamepad;

    // Driver Buttons
    private Trigger _driverAButton, _driverBButton, _driverXButton, _driverYButton;
    private Trigger _driverLeftBumper, _driverRightBumper, _driverStartButton, _driverBackButton;
    private Trigger _driverPOVUp, _driverPOVDown, _driverPOVLeft, _driverPOVRight;

    // Driver Axis Buttons
    private AxisButton _driverLeftTriggerButton, _driverRightTriggerButton;
    private AxisButton _driverLeftXAxis, _driverLeftYAxis, _driverRightXAxis, _driverRightYAxis;

    // Operator Buttons
    private Trigger _operatorAButton, _operatorBButton, _operatorXButton, _operatorYButton;
    private Trigger _operatorLeftBumper, _operatorRightBumper, _operatorStartButton, _operatorBackButton;
    private Trigger _operatorPOVUp, _operatorPOVDown, _operatorPOVLeft, _operatorPOVRight;

    // Operator Axis Buttons
    private AxisButton _operatorLeftTriggerButton, _operatorRightTriggerButton;
    private AxisButton _operatorLeftXAxis, _operatorLeftYAxis, _operatorRightXAxis, _operatorRightYAxis;

    public OI() {

        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);

        // Driver Buttons
        _driverAButton = new Trigger(_driverGamepad.getAButton());
        _driverBButton = new Trigger(_driverGamepad.getBButton());
        _driverXButton = new Trigger(_driverGamepad.getXButton());
        _driverYButton = new Trigger(_driverGamepad.getYButton());
        _driverLeftBumper = new Trigger(_driverGamepad.getLeftBumper());
        _driverRightBumper = new Trigger(_driverGamepad.getRightBumper());
        _driverStartButton = new Trigger(_driverGamepad.getStartButton());
        _driverBackButton = new Trigger(_driverGamepad.getBackButton());

        // Driver POV Buttons
        _driverPOVUp = new Trigger(() -> _driverGamepad.getPOV() == 0);
        _driverPOVDown = new Trigger(() -> _driverGamepad.getPOV() == 180);
        _driverPOVLeft = new Trigger(() -> _driverGamepad.getPOV() == 270);
        _driverPOVRight = new Trigger(() -> _driverGamepad.getPOV() == 90);

        // Driver Axis Buttons (Threshold-Based)
        _driverLeftTriggerButton = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05);
        _driverRightTriggerButton = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05);
        _driverLeftXAxis = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber(), 0.2);
        _driverLeftYAxis = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber(), 0.2);
        _driverRightXAxis = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber(), 0.2);
        _driverRightYAxis = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_Y.getNumber(), 0.2);

        // Operator Buttons
        _operatorAButton = new Trigger(_operatorGamepad.getAButton());
        _operatorBButton = new Trigger(_operatorGamepad.getBButton());
        _operatorXButton = new Trigger(_operatorGamepad.getXButton());
        _operatorYButton = new Trigger(_operatorGamepad.getYButton());
        _operatorLeftBumper = new Trigger(_operatorGamepad.getLeftBumper());
        _operatorRightBumper = new Trigger(_operatorGamepad.getRightBumper());
        _operatorStartButton = new Trigger(_operatorGamepad.getStartButton());
        _operatorBackButton = new Trigger(_operatorGamepad.getBackButton());

        // Operator POV Buttons
        _operatorPOVUp = new Trigger(() -> _operatorGamepad.getPOV() == 0);
        _operatorPOVDown = new Trigger(() -> _operatorGamepad.getPOV() == 180);
        _operatorPOVLeft = new Trigger(() -> _operatorGamepad.getPOV() == 270);
        _operatorPOVRight = new Trigger(() -> _operatorGamepad.getPOV() == 90);

        // Operator Axis Buttons (Threshold-Based)
        _operatorLeftTriggerButton = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), 0.05);
        _operatorRightTriggerButton = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), 0.05);
        _operatorLeftXAxis = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber(), 0.2);
        _operatorLeftYAxis = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber(), 0.2);
        _operatorRightXAxis = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_X.getNumber(), 0.2);
        _operatorRightYAxis = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber(), 0.2);
    }

    
    //this is where we map commands
    public void initializeButtons(Drivetrain drivetrain) {

        //this is where we map commands

        //_driverGamepad.getAButton().onTrue(new ShootAmp(indexer, shooter));
        //_driverGamepad.getYButton().onTrue(new ClimbUp(arm));
        _driverGamepad.getBButton().onTrue(new ResetIMU(drivetrain));
        _operatorGamepad.getAButton().onTrue(new Snap(drivetrain, new Rotation2d(Math.PI)));
        _operatorGamepad.getXButton().onTrue(new Snap(drivetrain, new Rotation2d(90)));
        _operatorGamepad.getYButton().onTrue(new Snap(drivetrain, new Rotation2d(0)));
        _operatorGamepad.getBButton().onTrue(new Snap(drivetrain, new Rotation2d(270)));
        _driverGamepad.getLeftBumper().whileTrue(new SlowMode(drivetrain, Constants.Drivetrain.DRIVE_KINEMATIC_LIMITS));

    }

    public double getDriveY() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = Helpers.applyDeadband(speed, Constants.Drivetrain.TRANSLATION_DEADBAND);
        return speed;
    }

    public double getDriveX() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = Helpers.applyDeadband(speed, Constants.Drivetrain.TRANSLATION_DEADBAND);
        return speed;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = Helpers.applyDeadband(speed, Constants.Drivetrain.ROTATION_DEADBAND);
        return speed;
    }


    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    public void rumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopRumbleDriver() {
        _driverGamepad.setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }

    public boolean getArmUp() {
        return _driverGamepad.getYButton().getAsBoolean();
    }

    public boolean getArmDown() {
        return _driverGamepad.getAButton().getAsBoolean();
    }

    public double getSnapHeading() {
        double heading;
        if (_driverGamepad.povUp(null).getAsBoolean()) {
            heading = 0.0;
        } else if (_driverGamepad.povDown(null).getAsBoolean()) {
            heading = 180.0;
        } else if (_driverGamepad.povLeft(null).getAsBoolean()) {
            heading = 90.0;
        } else if (_driverGamepad.povRight(null).getAsBoolean()) {
            heading = 270.0;
        } else {
            heading = 0.0;
        }
        return heading;
    }


}