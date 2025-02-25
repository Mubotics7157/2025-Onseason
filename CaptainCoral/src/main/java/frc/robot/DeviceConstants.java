package frc.robot;

public final class DeviceConstants {
    //====================CONTROLLER====================
    public static final int DRIVER_CONTROLLER_DEVICE_ID = 0;
    public static final int OPERATOR_CONTROLLER_DEVICE_ID = 1;
    public static final double JOYSTICK_DEADZONE_TOLERANCE = 0.05;
    public static final double DRIVER_CONTROLLER_RUMBLE = 1.0;

    //====================RIO CANBUS====================
    //Intake/Indexer
    public static final int INTAKE_WRIST_MOTOR_DEVICE_ID = 14;
    public static final int INTAKE_ROLLER_MOTOR_DEVICE_ID = 15;
    public static final int INDEXER_MOTOR_DEVICE_ID = 16;

    public static final int INTAKE_WRIST_THROUGH_BORE_PORT = 0;
    //public static final int INDEXER_PHOTOELECTRIC_PORT = 2;

    //End Effector
    public static final int END_EFFECTOR_WRIST_MOTOR_DEVICE_ID = 17;
    public static final int END_EFFECTOR_TOP_ROLLER_MOTOR_DEVICE_ID = 18;
    public static final int END_EFFECTOR_BOTTOM_ROLLER_MOTOR_DEVICE_ID = 19;

    public static final int END_EFFECTOR_WRIST_THROUGH_BORE_PORT = 2;
    public static final int END_EFFECTOR_PHOTOELECTRIC_PORT = 8;

    //Elevator
    public static final int ELEVATOR_MASTER_MOTOR_DEVICE_ID = 20;
    public static final int ELEVATOR_SLAVE_MOTOR_DEVICE_ID = 21;

    //Climb
    public static final int CLIMB_MASTER_MOTOR_DEVICE_ID = 22;
    public static final int CLIMB_SLAVE_MOTOR_DEVICE_ID = 23;
}