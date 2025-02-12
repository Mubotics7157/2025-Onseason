package frc.robot;

public final class DeviceConstants {
    //====================CONTROLLER====================
    public static final int DRIVER_CONTROLLER_DEVICE_ID = 0; //Final
    public static final double JOYSTICK_DEADZONE_TOLERANCE = 0.05; //Final

    //====================RIO CANBUS====================
    //Intake/Indexer
    public static final int INTAKE_WRIST_MOTOR_DEVICE_ID = 14;
    public static final int INTAKE_ROLLER_MOTOR_DEVICE_ID = 15;

    public static final int INDEXER_SENSOR_PORT = 0;
    public static final int INTAKE_INDEXER_MASTER_MOTOR_DEVICE_ID = 16;
    public static final int INTAKE_INDEXER_SLAVE_MOTOR_DEVICE_ID = 17;

    //End Effector
    public static final int END_EFFECTOR_WRIST_MASTER_MOTOR_DEVICE_ID = 18;
    public static final int END_EFFECTOR_WRIST_SLAVE_MOTOR_DEVICE_ID = 19;

    public static final int END_EFFECTOR_SENSOR_PORT = 7;
    public static final int END_EFFECTOR_TOP_MOTOR_DEVICE_ID = 20;
    public static final int END_EFFECTOR_BOTTOM_MOTOR_DEVICE_ID = 21;

    //Elevator
    public static final int ELEVATOR_MASTER_MOTOR_DEVICE_ID = 22;
    public static final int ELEVATOR_SLAVE_MOTOR_DEVICE_ID = 23;

    //Climb
    public static final int CLIMB_MASTER_MOTOR_DEVICE_ID = 24;
    public static final int CLIMB_SLAVE_MOTOR_DEVICE_ID = 25;
}