package frc.robot;
import static edu.wpi.first.units.Units.*;

public class Constants {
    //====================General====================
    public static final double Absolute_Zero = 0.0;
    public static final double PID_Setpoint_Tolerance = 0.12;

    //====================Drivetrain====================
    //Default
    public static final double DrivetrainMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double DrivetrainMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static final double Drivetrain_Speed_Multiplier = 1.0;
    public static final double Drivetrain_Turn_Multiplier = 1.0;

    //Auto Align
    public static final double Drivetrain_X_Pose_kP = 0.01;
    public static final double Drivetrain_FB_kP = 0.015;
    public static final double Drivetrain_LR_kP = 0.0055;
    public static final double Drivetrain_Rot_kP = 0.02;
    public static final double Left_Pole_Setpoint = 25.0;
    public static final double Right_Pole_Setpoint = -30.0;
    public static final double Square_Up_Setpoint = 0.0;
    public static final double Drivetrain_Auto_Align_Tolerance = 0.025;

    //Elevator Placement
    public static final double Drivetrain_Elevator_Speed_Multiplier = 0.15;
    public static final double Drivetrain_Elevator_Turn_Multiplier = 0.7;
    public static final double Drivetrain_Elevator_Tolerance = 1.0;

    //====================Intake====================
    //Through Bore Encoder
    public static final double Intake_Wrist_Through_Bore_Gear_Ratio = (50/8) * (62/16);
    public static final double Intake_Wrist_Through_Bore_Offset = 0.3; //0.94312122289

    //Intake Motion Magic
    public static final double Intake_Wrist_kG = 0.0; //2.0
    public static final double Intake_Wrist_kP = 0.0;
    public static final double Intake_Wrist_kI = 0.0; //0.03
    public static final double Intake_Wrist_kD = 0.0;
    public static final double Intake_Wrist_Velocity = 0.0; //75.0
    public static final double Intake_Wrist_Acceleration = 0.0; //150

    //Intake Current Limits
    public static final double Intake_Wrist_Current_Limit = 80.0;
    public static final double Intake_Roller_Current_Limit = 100.0;

    //Intake Speeds
    public static final double Intake_Ground_Run_Speed = 0.75;
    public static final double Outake_Ground_Run_Speed = -0.2;

    //Intake Setpoints
    public static final double Intake_Zero_Setpoint = 8.8; // OLD: 28
    public static final double Intake_Ground_Deploy_Setpoint = 0.8; // OLD: 18

    //====================End Effector====================
    //End Effector Motion Magic
    public static final double End_Effector_Wrist_kP = 2.0;
    public static final double End_Effector_Wrist_kI = 0.0;
    public static final double End_Effector_Wrist_kD = 0.03;
    public static final double End_Effector_Wrist_Velocity = 150.0; //75
    public static final double End_Effector_Wrist_Acceleration = 300.0; //150

    //End Effector Current Limits
    public static final double End_Effector_Wrist_Current_Limit = 80.0;
    public static final double End_Effector_Roller_Current_Limit = 120.0;

    //~~~~~End Effector Speeds~~~~~
    //TEMPORARY
    //Coral
    public static final double End_Effector_Ground_Intake_Speed = 0.3;
    public static final double End_Effector_Ground_Outake_Speed = -0.4;
    public static final double End_Effector_Coral_Station_Intake_Speed = 0.0;
    public static final double End_Effector_Score_L1_Coral_Speed = 0.3;
    public static final double End_Effector_Score_L2_L3_L4_Speed = 0.75; //0.175

    //Algae
    public static final double End_Effector_Algae_Intake_Speed = 1.0;
    public static final double End_Effector_Algae_Score_Speed = -1.0;

    //~~~~~End Effector Setpoints~~~~~
    //Coral
    public static final double End_Effector_Wrist_Zero_Setpoint = 0.0;
    public static final double End_Effector_Wrist_Coral_Ground_Setpoint = 2.56;
    public static final double End_Effector_Wrist_L1_Score_Setpoint = 3.0;
    public static final double End_Effector_Wrist_L2_L3_Score_Setpoint = 14.5;
    public static final double End_Effector_Wrist_L4_Score_Setpoint = 15.5;
    public static final double End_Effector_Wrist_Coral_Station_Setpoint = 0.02;

    //Algae  
    public static final double End_Effector_Wrist_Algae_Remove_Setpoint = 31.0;
    public static final double End_Effector_Wrist_Algae_Ground_Setpoint = 37.0;
    public static final double End_Effector_Wrist_Algae_Stow_Setpoint = 15.5; //4.0
    public static final double End_Effector_Wrist_Processor_Score_Setpoint = 31.0;
    public static final double End_Effector_Wrist_Net_Score_Setpoint = 21.0;

    public static final double End_Effector_Wrist_Climb_Start_Setpoint = 1.0;
    public static final double End_Effector_Wrist_Climb_End_Setpoint = 12.00;

    //====================Elevator====================
    //Elevator Motion Magic
    public static final double Elevator_kG = -0.3;
    public static final double Elevator_kP = 2.25;
    public static final double Elevator_kI = 0.0;
    public static final double Elevator_kD = 0.03;
    public static final double Elevator_Velocity = 100.0;
    public static final double Elevator_Acceleration = 200.0;

    //Elevator Curent Limit
    public static final double Elevator_Current_Limit = 80.0;

    //~~~~~Elevator Setpoints~~~~~
    //Coral
    public static final double Elevator_Zero_Setpoint = 0.0;
    public static final double Elevator_Ground_Coral_Setpoint = 0.0;
    public static final double Elevator_Coral_Station_Setpoint = 6.5;
    public static final double Elevator_L1_Setpoint = 3.5;
    public static final double Elevator_L2_Setpoint = 4.0; //6.15
    public static final double Elevator_L3_Setpoint = 18.0; //13.5
    public static final double Elevator_L4_Setpoint = 24.14; //25.85

    //Algae
    public static final double Elevator_Bottom_Algae_Setpoint = 13.5;
    public static final double Elevator_Top_Algae_Setpoint = 21.0;
    public static final double Elevator_Ground_Algae_Setpoint = 0.0;
    public static final double Elevator_Processor_Score_Setpoint = 0.0;
    public static final double Elevator_Net_Score_Setpoint = 24.14;

    public static final double Elevator_Climb_Setpoint = 16.5;

    //====================Climb====================
    //Climb Current Limit
    public static final double Climb_Current_Limit = 120.0;

    //Climb Speeds
    public static final double Climb_Up_Speed = 1.0;
    public static final double Climb_Down_Speed = -1.0;
}