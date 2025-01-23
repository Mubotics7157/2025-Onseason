package frc.robot;

public final class DeviceConstants {
    //====================CONTROLLER====================
    public static final int CONTROLLER_DEVICE_ID = 0; //Final
    public static final double DEADZONE_TOLERANCE = 0.05; //Final

    public static final int LEFT_STICK_HORIZONTAL_AXIS = 0; //Final
    public static final int LEFT_STICK_VERTICAL_AXIS = 1; //Final
    public static final int RIGHT_STICK_HORIZONTAL_AXIS = 4; //Final
    public static final int RIGHT_STICK_VERTICAL_AXIS = 5; //Final

    public static final int INTAKE_BUTTON = 5;
    public static final int SCORE_BUTTON = 6;

    public static final int ELEVATOR_L1_BUTTON = 1; //Final
    public static final int ELEVATOR_L2_BUTTON = 2; //Final
    public static final int ELEVATOR_L3_BUTTON = 3; //Final
    public static final int ELEVATOR_L4_BUTTON = 4; //Final

    //====================SWERVE CANBUS====================
    public static final int DRIVE_FL_MOTOR_DEVICE_ID = 1; 
    public static final int TURN_FL_MOTOR_DEVICE_ID = 2;
    public static final int DRIVE_FL_ENCODER_DEVICE_ID = 3;

    public static final int DRIVE_FR_MOTOR_DEVICE_ID = 4;
    public static final int TURN_FR_MOTOR_DEVICE_ID = 5;
    public static final int DRIVE_FR_ENCODER_DEVICE_ID = 6;

    public static final int DRIVE_BL_MOTOR_DEVICE_ID = 7;
    public static final int TURN_BL_MOTOR_DEVICE_ID = 8;
    public static final int DRIVE_BL_ENCODER_DEVICE_ID = 9;

    public static final int DRIVE_BR_MOTOR_DEVICE_ID = 10;
    public static final int TURN_BR_MOTOR_DEVICE_ID = 11;
    public static final int DRIVE_BR_ENCODER_DEVICE_ID = 12;

    public static final int GYRO_PIGEON_DEVICE_ID = 13;

    //====================RIO CANBUS====================
    public static final int INTAKE_WRIST_MOTOR_DEVICE_ID = 14;
    public static final int INTAKE_ROLLER_MOTOR_DEVICE_ID = 15;
    public static final int INTAKE_INDEXER_MOTOR_DEVICE_ID = 16;

    public static final int END_EFFECTOR_WRIST_MOTOR_DEVICE_ID = 17;
    public static final int END_EFFECTOR_TOP_MOTOR_DEVICE_ID = 18;
    public static final int END_EFFECTOR_BOTTOM_MOTOR_DEVICE_ID = 19;

    public static final int ELEVATOR_MASTER_MOTOR_DEVICE_ID = 20;
    public static final int ELEVATOR_SLAVE_MOTOR_DEVICE_ID = 21;

    public static final int CLIMB_MOTOR_DEVICE_ID = 22;

    //====================PNEUMATICS====================
    public static final int COMPRESSOR_DEVICE_ID = 0;
    public static final int CLAW_SOLENOID_1_FORWARD_CHANNEL = 0;
    public static final int CLAW_SOLENOID_1_REVERSE_CHANNEL = 1; 
    public static final int CLAW_SOLENOID_2_FORWARD_CHANNEL = 2;
    public static final int CLAW_SOLENOID_2_REVERSE_CHANNEL = 3;
}
