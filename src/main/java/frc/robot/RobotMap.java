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

        public static final int ELEVATOR_MOTOR_LEFT_CAN = 23;
        public static final int ELEVATOR_MOTOR_RIGHT_CAN = 22;

        public static final int CLIMB_MOTOR_CAN = 24;

        public static final int ARM_MOTOR_CAN = 26;

        public static final int ENDEFFECTOR_MOTOR_TOP_CAN = 20;
        public static final int ENDEFFECTOR_MOTOR_BOTTOM_CAN = 21;

        public static final int PIGEON_CAN = 18;
    }

    public static class DIO {
        // Button IDs
        public static final int ENDEFFECTOR_PROX_SENSOR_ID = 0;
    }

    public static class PWM {

    }

    public static class Analog {

    }


}

