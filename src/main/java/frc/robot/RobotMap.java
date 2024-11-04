package frc.robot;

public class RobotMap {


    public static class CAN {
        // CAN IDs
        public static final int BR_STEER_CAN = 13; // Changed to make room for PDH CAN ID of 1
        public static final int FR_STEER_CAN = 14; // Made room for talon's can ID of 2 lol
        public static final int FL_STEER_CAN = 3;
        public static final int BL_STEER_CAN = 4;
        public static final int BR_DRIVE_CAN = 5;
        public static final int FR_DRIVE_CAN = 6;
        public static final int FL_DRIVE_CAN = 7;
        public static final int BL_DRIVE_CAN = 8;

        // Intake/Shooter IDs
        public static final int INTAKE_MOTOR_CAN = 9;
        public static final int LOADING_MOTOR_CAN = 10;
        public static final int SHOOTER_MOTOR_CAN = 11;
        public static final int ANGLE_ALIGNMENT_MOTOR_CAN = 2;
        public static final int ANGLE_ALIGNMENT_ENCODER_CAN = 15;

    }

    public static class DIO {
        // Button IDs
        public static final int SHOOTER_IS_LOADED_BUTTON_ID = 0;
        public static final int IS_LOWEST_ANGLE_BUTTON_ID = 1;
        public static final int IS_HIGHEST_ANGLE_BUTTON_ID = 2;

    }

    public static class PWM {

    }

    public static class Analog {

    }


}

