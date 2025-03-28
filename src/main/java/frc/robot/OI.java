package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.AxisButton;
import frc.robot.Utils.Gamepad;
import frc.robot.Utils.Helpers;
import frc.robot.Utils.RobotState;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.Autonomous.AutonomousScoreApproach;
import frc.robot.commands.Drive.ResetIMU;
import frc.robot.commands.Elevator.ElevatorIntake;
import frc.robot.commands.Elevator.ElevatorL2;
import frc.robot.commands.Elevator.ElevatorL3;
import frc.robot.commands.Elevator.ElevatorL4;
import frc.robot.commands.EndEffector.*;
import frc.robot.commands.Score.Intake;
import frc.robot.commands.Score.ScoreL1;
import frc.robot.commands.Score.ScoreL2;
import frc.robot.commands.Score.ScoreL3;
import frc.robot.commands.Score.ScoreL4;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.EndEffectorOuttake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.EndEffector;



public class OI {
    
    private final Drivetrain _drivetrain;
    private final Elevator _elevator;
    private final Arm _arm;
   // private final Climb _climb;
    private final EndEffector _endEffector;
    

 

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

    public OI(Drivetrain drivetrain, Elevator elevator, Arm arm, EndEffector endEffector) {
        
        _drivetrain = drivetrain;
        _elevator = elevator;
        _arm = arm;
       // _climb = new Climb();
        _endEffector =endEffector;
        
        

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
    public void initializeButtons() {
        _driverAButton.onTrue(new ElevatorIntake(_elevator, _arm));
        _driverBButton.onTrue(new ResetIMU(_drivetrain));
        _driverXButton.onTrue(new ElevatorL2(_elevator, _arm));
        _driverYButton.onTrue(new ElevatorL3(_elevator, _arm));
        _driverLeftBumper.onTrue(new ElevatorL4(_elevator, _arm));
        _driverRightBumper.onTrue(new ArmIntake(_arm));
        _driverStartButton.onTrue(new ArmScoreL3(_arm));
        _driverBackButton.onTrue(Commands.none());

        // Driver POV Buttons
        _driverPOVUp.onTrue(Commands.none());
        _driverPOVDown.onTrue(Commands.none());
        _driverPOVLeft.onTrue(Commands.none());
        _driverPOVRight.onTrue(Commands.none());

        // Operator Buttons
        /*
        _operatorAButton.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L1); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.RIGHT);}));
        _operatorBButton.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L3); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.RIGHT);}));
        _operatorXButton.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L2); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.RIGHT);}));
        _operatorYButton.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L4); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.RIGHT);}));
        */
        _operatorAButton.onTrue(new Intake(_elevator, _arm));
        _operatorBButton.onTrue(new ScoreL3(_elevator, _arm));
        _operatorXButton.onTrue(new ScoreL2(_elevator, _arm));
        _operatorYButton.onTrue(new ScoreL4(_elevator, _arm));

        _operatorLeftBumper.onTrue(new EndEffectorIntake( _endEffector));
        _operatorRightBumper.onTrue(new EndEffectorOuttake( _endEffector));
        _operatorStartButton.onTrue(new StopWheels(_endEffector));
        _operatorBackButton.onTrue(Commands.none());

        // Operator POV Buttons
        /*
        _operatorPOVUp.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L4); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.LEFT);}));
        _operatorPOVDown.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L1); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.LEFT);}));
        _operatorPOVLeft.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L2); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.LEFT);}));
        _operatorPOVRight.onTrue(Commands.run(() -> {RobotState.getInstance().setScoringCoralLevel(Constants.CoralLevel.L3); RobotState.getInstance().setScoringReefDirection(Constants.ReefDirection.LEFT);}));
        */

        /*
        new Trigger(_driverLeftTriggerButton::get).onTrue(new IntakeCoralFunnel(_endEffector, _arm, _elevator));
        new Trigger(_driverRightTriggerButton::get).onTrue(Commands.run(() -> {
            if (RobotState.getInstance().getScoringReefDirection() == Constants.ReefDirection.LEFT) {
                switch (RobotState.getInstance().getScoringCoralLevel()) {
                    case L1: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "LEFT_CAMERA"), new PlaceL1(_endEffector, _arm, _elevator)).schedule(); break;
                    case L2: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "LEFT_CAMERA"), new PlaceL2(_endEffector, _arm, _elevator)).schedule(); break;
                    case L3: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "LEFT_CAMERA"), new PlaceL3(_endEffector, _arm, _elevator)).schedule(); break;
                    case L4: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "LEFT_CAMERA"), new PlaceL4(_endEffector, _arm, _elevator)).schedule(); break;
                }
            } else if (RobotState.getInstance().getScoringReefDirection() == Constants.ReefDirection.RIGHT) {
                switch (RobotState.getInstance().getScoringCoralLevel()) {
                    case L1: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "RIGHT_CAMERA"), new PlaceL1(_endEffector, _arm, _elevator)).schedule(); break;
                    case L2: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "RIGHT_CAMERA"), new PlaceL2(_endEffector, _arm, _elevator)).schedule(); break;
                    case L3: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "RIGHT_CAMERA"), new PlaceL3(_endEffector, _arm, _elevator)).schedule(); break;
                    case L4: Commands.sequence(new ArmAvoidElevator(_arm), new AutonomousScoreApproach(_drivetrain, "RIGHT_CAMERA"), new PlaceL4(_endEffector, _arm, _elevator)).schedule(); break;
                }
            }
        }));
        */
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

    public double getElevatorX() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber());

        speed = Helpers.applyDeadband(speed, Constants.Elevator.ELEVATOR_DEADBAND);
        return speed;
    }

    public double getArmX() {
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = Helpers.applyDeadband(speed, Constants.Arm.ARM_DEADBAND);

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