// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Jack Rubiralta was here!
//Linus Krenkel is more mature and responsable then jackjack
package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.subsystems.SwerveModule.ModuleConfiguration;


public class Constants {
    public static final double EPSILON = 1e-9;
    public static final double UPDATE_PERIOD = 0.02;
    //public static final RobotConfig

    public static class Drivetrain {
        public static final double WHEEL_DIAMETER = 0.0762;


        public static final double TRIGGER_DEAD_ZONE = 0.2; // Zero to one
        public static final double MANUAL_ARM_MOVE_SPEED = 0.0075;

        // Swerve Module Constants
        public static final double MIN_TURNING_SPEED = 0.05; // Radians per second
        public static final double MAX_TURING_SPEED = 3.5; // Radians per second

        public static final boolean IS_INVERTED = true;

        public static final int PINION_TEETH = 13;

        public static final double ABS_ENCODER_CONVERSION = 360; // CAN SPARK Default
        public static final double RELATIVE_ENCODER_CONVERSION = 46.5; //93/2 I think default
        public static final double FULL_ROTATION = 1; // 2

        public static final double BUMP_DEGREES = 7.0;
        public static ModuleConfiguration SOUTH_EAST_CONFIG = new ModuleConfiguration();
        public static ModuleConfiguration NORTH_EAST_CONFIG = new ModuleConfiguration();
        public static ModuleConfiguration NORTH_WEST_CONFIG = new ModuleConfiguration();
        public static ModuleConfiguration SOUTH_WEST_CONFIG = new ModuleConfiguration();
        public static final double DRIVING_REDUCTION = (45.0 * 22) / (PINION_TEETH * 15);
        public static final double DRIVE_POSITION_FACTOR = ((WHEEL_DIAMETER * Math.PI)
                / DRIVING_REDUCTION); // meters
        public static final double DRIVE_VELOCITY_FACTOR = ((WHEEL_DIAMETER * Math.PI)
                / DRIVING_REDUCTION) / 60.0; // meters per second
        public static final double STEER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double STEER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second
        public static final double POSITION_WRAPPING_MIN_INPUT = 0; // radians
        public static final double POSITION_WRAPPING_MAX_INPUT = STEER_POSITION_FACTOR; // radians
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode STEER_IDLE_MODE = IdleMode.kBrake;

        public static final double SPEED_MOD = 1.0;
        public static final double MAX_ANG_VEL = 10.0 * SPEED_MOD;
        //public static final double MAX_DRIVE_SPEED_MPS = (Units.rotationsPerMinuteToRadiansPerSecond(6784) / DRIVING_REDUCTION * (WHEEL_DIAMETER / 2));
        public static final double MAX_DRIVE_SPEED_MPS = 10.0;
        public static final double TRANSLATION_DEADBAND = 0.05;
        public static final double ROTATION_DEADBAND = 0.05;


        public static final double MOTOR_MAX_OUTPUT = 1;
        public static final double MOTOR_MIN_OUTPUT = -1;

        public static final double HEADING_TOLERANCE = Units.degreesToRadians(1.5); // rad
        public static final double FREE_SPEED_RPS = 5676 / 60;

        // Robot Physical Constants
        public static final double WHEELBASE = 0.6985; // Meters, distance between front and back
        public static final double TRACKWIDTH = 0.6223; // Meters, distance between left and right

        public static final double SWERVE_NS_POS = WHEELBASE / 2;
        public static final double SWERVE_WE_POS = TRACKWIDTH / 2;


        // Steering PID
        public static final double DRIVE_KP = 0.2;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.07;
        public static final double DRIVE_FF = 0.0 / FREE_SPEED_RPS;
        // Turning PID
        public static final double STEER_KP = 1.8;
        public static final double STEER_KI = 0.0;
        public static final double STEER_KD = 0.05;
        public static final double STEER_FF = 0.0;
        // Autonomous Drive PID
        public static final double AUTO_KP = 0.2;
        public static final double AUTO_KI = 0.0;
        public static final double AUTO_KD = 0.00;
        // Autonomous Constants
        public static final double AUTONOMOUS_POSITION_MAX_ERROR = 0.04; // Meters
        public static final KinematicLimits DRIVE_KINEMATIC_LIMITS = new KinematicLimits();
        public static final KinematicLimits SLOW_MODE_KINEMATIC_LIMITS = new KinematicLimits();
        public static final long DISABLE_TIME = 500;
        public static final double HEADING_kI = 0;
        public static final double HEADING_kP = 8.0;
        public static final double HEADING_kD = 1.0;
        public static final int STEER_CURRENT_LIMIT = 10;
        public static final int DRIVE_CURRENT_LIMIT = 40;

        //Encoder Offset set manually

        static {
            SOUTH_EAST_CONFIG.moduleName = "South East";

            SOUTH_EAST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, -SWERVE_WE_POS); // -,-

            SOUTH_EAST_CONFIG.encoderInverted = false;
            SOUTH_EAST_CONFIG.encoderOffset = Math.PI * (1.0 / 2.0);
        }

        static {
            NORTH_EAST_CONFIG.moduleName = "North East";

            NORTH_EAST_CONFIG.position = new Translation2d(SWERVE_NS_POS, -SWERVE_WE_POS); // +,-

            NORTH_EAST_CONFIG.encoderInverted = false;
            NORTH_EAST_CONFIG.encoderOffset = Math.PI * (4.0 / 2.0);

        }

        static {
            NORTH_WEST_CONFIG.moduleName = "North West";

            NORTH_WEST_CONFIG.position = new Translation2d(SWERVE_NS_POS, SWERVE_WE_POS); // +,+

            NORTH_WEST_CONFIG.encoderInverted = false;
            NORTH_WEST_CONFIG.encoderOffset = Math.PI * (3.0 / 2.0);
        }

        static {
            SOUTH_WEST_CONFIG.moduleName = "South West";

            SOUTH_WEST_CONFIG.position = new Translation2d(-SWERVE_NS_POS, SWERVE_WE_POS); // -,+

            SOUTH_WEST_CONFIG.encoderInverted = false;
            SOUTH_WEST_CONFIG.encoderOffset = Math.PI * (2.0 / 2.0);
        }

        static {
            DRIVE_KINEMATIC_LIMITS.maxDriveAcceleration = Double.MAX_VALUE;
            DRIVE_KINEMATIC_LIMITS.maxDriveVelocity = MAX_DRIVE_SPEED_MPS;
            DRIVE_KINEMATIC_LIMITS.maxSteeringVelocity = Double.MAX_VALUE;
        }

        static {
            SLOW_MODE_KINEMATIC_LIMITS.maxDriveAcceleration = Double.MAX_VALUE;
            SLOW_MODE_KINEMATIC_LIMITS.maxDriveVelocity = 3.0;
            SLOW_MODE_KINEMATIC_LIMITS.maxSteeringVelocity = 8.0;
        }

    }

    public static class Shooter {
        // Encoder Constants


        public static final double SHOOTER_ANGLE_CONVERSION = 1; // Ratio between encoder and angle of shooter // TODO: Needs to be measured
        public static final double LAUNCH_SPEED_CONVERSION = 3; // Ratio between encoder and angle of shooter // TODO: Needs to be measured
        public static final double SHOOTING_ANGLE_ERROR = 3; // Degrees

        // Field Constants
        public static final double GRAVITY = 9.81; // Meters per second^2
        public static final double GOAL_HEIGHT = 2.44; // Meters
        public static final long AMP_SCORE_TIME = 1500; //ms
        public static final long SHOOT_WOOF_DELAY = 2000; //ms
        public static final double IDLE_SHOOTER_SPEED = 0.0;


        // shooter PID
        public static final double SHOOTER_KP = 2.0;
        public static final double SHOOTER_KI = 0.0;
        public static final double SHOOTER_KD = 0.00;

        public static final double SHOOTER_EXIT_VELOCITY = 8.0; // Meters per second
        public static final double ROBOT_SHOOTER_HEIGHT = 1; // Meters
        public static final double SHOOTER_SUBWOOFER_SPEED = 1.0;
        public static final double SHOOTER_AMP_SPEED = -0.5;
    }


    public static class In {

        public static final double LOADING_SPEED = 0.45;
        public static final double SHOOT_STATIC_SPEED = 1;
        public static final long SHOOT_DELAY = 500; //ms

        public static final double BACKOUT_SPEED = -0.4;
        public static final long BACKOUT_TIME = 300; // ms

        public static final double EJECT_SPEED = -1.0;

        public static final double INTAKE_SPEED = 0.80;
    }

    public static class Arm {
        public static final double AMP_SCORING_ANGLE = -0.308; // Rads
        public static final double ARM_INTAKE_ANGLE = 0.232; // Rad
        public static final double ARM_MAX_ANGLE = -0.423; // TODO change this to what it is

        // Shooter Angle Alignment PID
        public static final double SHOOTING_ANGLE_KP = 7.0;
        public static final double SHOOTING_ANGLE_KI = 0.000;
        public static final double SHOOTING_ANGLE_KD = 0.5;
        public static final double TOLERANCE = 0.05;

        public static final double SHOOT_SUB_ANGLE = 0.00; //TODO change this
        public static final double MANUAL_ARM_MOVE_SPEED = .45;

        public static final double ARM_CLIMB_ANGLE = 0.0; //I do not know what it is supposed to be since it dissappeard in merging process. we need to figure out....
    }

    public static class Autonomous {
        public static final boolean IS_COMPETITION = false;
        public static final String[] PATH_PLANNER_PATHS = {
                "Hello_Path",
                "Hello_Path2",
                "Hello_Path3"
        };

        public static class Score {
            public static final double DRIVE_X_KP = 0.5;
            public static final double DRIVE_X_KI = 0.0;
            public static final double DRIVE_X_KD = 0.02;

            public static final double DRIVE_Y_KP = 0.5;
            public static final double DRIVE_Y_KI = 0.0;
            public static final double DRIVE_Y_KD = 0.02;

            public static final double ROTATION_KP = 0.5;
            public static final double ROTATION_KI = 0.0;
            public static final double ROTATION_KD = 0.02;

            public static final double DISTANCE_FACING_X = 1; // space between front of camera and april tag in meters

        }
    }

    public static enum CoralLevel {
        L1,
        L2,
        L3,
        L4
    }

    public static enum ReefDirection {
        LEFT,
        RIGHT
    }
}
