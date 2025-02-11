package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KinematicsConstants { //FINAL KEYWORD???
    //====================General====================
    public static final double Absolute_Zero = 0.0;
    public static final double Jog_Speed_Multiplier = -0.125;
    public static final double PID_Setpoint_Tolerance = 0.1;

    //====================Drivetrain====================
    public static final double Drivetrain_Speed_Multiplier = 0.50;
    public static final double Drivetrain_Turn_Multiplier = 0.5;

    public static final double Left_Pole_Setpoint = 20.0; //Square Up: 18.5 | Triangle: 10.0
    public static final double Right_Pole_Setpoint = -23.0; //Square Up: -21.0 | Triangle: 20.0
    public static final double Square_Up_Setpoint = 0.0;

    public static final double Drivetrain_FB_kP = 0.015; //0.015
    public static final double Drivetrain_LR_kP = 0.00375; //0.0075

    public static final double Drivetrain_Auto_Align_Tolerance = 0.025;

    //====================Intake====================
    //Intake Motion Magic
    public static final double Intake_Wrist_kS = 0.0;
    public static final double Intake_Wrist_kV = 0.12;
    public static final double Intake_Wrist_kA = 0.05;
    public static final double Intake_Wrist_kP = 10.0;
    public static final double Intake_Wrist_kI = 0.0;
    public static final double Intake_Wrist_kD = 0.0;
    public static final double Intake_Wrist_Velocity = 32.0;
    public static final double Intake_Wrist_Acceleration = 64.0;
    public static final double Intake_Wrist_Jerk = 128.0;

    //Intake Current Limits
    public static final double Intake_Wrist_Current_Limit = 80.0;
    public static final double Intake_Roller_Current_Limit = 80.0;

    //Intake Speeds
    public static final double Intake_Ground_Run_Speed = 0.8;
    public static final double Outake_Ground_Run_Speed = -0.8;

    //Intake Setpoints
    public static final double Intake_Zero_Setpoint = 0.0;
    public static final double Intake_Ground_Deploy_Setpoint = 8.5;

    //====================End Effector====================
    //End Effector Motion Magic
    public static final double End_Effector_Wrist_kS = 0.0;
    public static final double End_Effector_Wrist_kV = 0.12;
    public static final double End_Effector_Wrist_kA = 0.05;
    public static final double End_Effector_Wrist_kP = 10.0;
    public static final double End_Effector_Wrist_kI = 0.0;
    public static final double End_Effector_Wrist_kD = 0.0;
    public static final double End_Effector_Wrist_Velocity = 32.0;
    public static final double End_Effector_Wrist_Acceleration = 64.0;
    public static final double End_Effector_Wrist_Jerk = 128.0;

    //End Effector Current Limits
    public static final double End_Effector_Wrist_Current_Limit = 80.0;
    public static final double End_Effector_Roller_Current_Limit = 80.0;

    //End Effector Speeds
    public static final double End_Effector_Score_Speed = -0.4;
    public static final double End_Effector_Ground_Intake_Speed = -0.15;
    public static final double End_Effector_Ground_Outake_Speed = 0.15;
    public static final double End_Effector_Coral_Station_Intake_Speed = 0.15;

    //End Effector Setpoints
    public static final double End_Effector_Wrist_Zero_Setpoint = 0.1;

    public static final double End_Effector_Wrist_L1_Score_Setpoint = 3.0;
    public static final double End_Effector_Wrist_L2_L3_Score_Setpoint = 4.25;
    public static final double End_Effector_Wrist_L4_Score_Setpoint = 5.5;

    public static final double End_Effector_Wrist_Coral_Station_Setpoint = 0.02;
    public static final double End_Effector_Wrist_Algae_Remove_Setpoint = 17.0;
    public static final double End_Effector_Wrist_Processor_Score_Setpoint = 19.0;
    public static final double End_Effector_Wrist_Barge_Score_Setpoint = 14.0;

    //====================Elevator====================
    //Elevator Motion Magic
    public static final double Elevator_kS = 0.0;
    public static final double Elevator_kV = 0.12;
    public static final double Elevator_kA = 0.05;
    public static final double Elevator_kP = 9.0;
    public static final double Elevator_kI = 0.0;
    public static final double Elevator_kD = 0.0;
    public static final double Elevator_Velocity = 20.0;
    public static final double Elevator_Acceleration = 40.0;
    public static final double Elevator_Jerk = 80.0;

    //Elevator Curent Limit
    public static final double Elevator_Current_Limit = 80.0;

    //Elevator Setpoints
    public static final double Elevator_Zero_Setpoint = 0.0;
    public static final double Elevator_Coral_Station_Setpoint = 6.5;

    public static final double Elevator_L1_Setpoint = 3.0;
    public static final double Elevator_L2_Setpoint = 8.15;
    public static final double Elevator_L3_Setpoint = 14.8;
    public static final double Elevator_L4_Setpoint = 26.5;

    public static final double Elevator_Bottom_Algae_Setpoint = 15.0;
    public static final double Elevator_Top_Algae_Setpoint = 20.5;
    public static final double Elevator_Processor_Score_Setpoint = 0.0;
    public static final double Elevator_Barge_Score_Setpoint = 26.5;

    //====================Climb====================
    //Climb Current Limit
    public static final double Climb_Current_Limit = 120.0;

    //Climb Speeds
    public static final double Climb_Up_Speed = 0.8;
    public static final double Climb_Down_Speed = -0.8;
}