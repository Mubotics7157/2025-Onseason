package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.*;

public class PhysConstants { //FINAL KEYWORD???
    //====================General====================
    public static final double Absolute_Zero = 0.0;
    public static final double Jog_Speed_Multiplier = 0.125;
    public static final double PID_Setpoint_Tolerance = 0.12;

    //====================Drivetrain====================
    public static final double DrivetrainMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double DrivetrainMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static final double Drivetrain_X_Pose_kP = 0.001;

    public static final double Drivetrain_Speed_Multiplier = 0.8;
    public static final double Drivetrain_Turn_Multiplier = 1.0;

    public static final double Drivetrain_Elevator_Speed_Multiplier = 0.15;
    public static final double Drivetrain_Elevator_Turn_Multiplier = 0.7;
    public static final double Drivetrain_Elevator_Tolerance = 1.0;

    public static final double Left_Pole_Setpoint = 25.0;
    public static final double Right_Pole_Setpoint = -30.0;
    public static final double Square_Up_Setpoint = 0.0;

    public static final double Drivetrain_FB_kP = 0.015;
    public static final double Drivetrain_LR_kP = 0.0055;
    public static final double Drivetrain_Rot_kP = 0.02;

    public static final double Drivetrain_Auto_Align_Tolerance = 0.025;

    //====================Intake====================
    //Intake Motion Magic
    public static final double Intake_Wrist_kS = 0.0;
    public static final double Intake_Wrist_kV = 0.12;
    public static final double Intake_Wrist_kA = 0.05;
    public static final double Intake_Wrist_kP = 10.0;
    public static final double Intake_Wrist_kI = 0.0;
    public static final double Intake_Wrist_kD = 0.0;
    public static final double Intake_Wrist_Velocity = 256.0; //64
    public static final double Intake_Wrist_Acceleration = 512.0; //128
    public static final double Intake_Wrist_Jerk = 1024.0; //256

    //Intake Current Limits
    public static final double Intake_Wrist_Current_Limit = 80.0;
    public static final double Intake_Roller_Current_Limit = 80.0;

    //Intake Speeds
    public static final double Intake_Ground_Run_Speed = -0.6;
    public static final double Outake_Ground_Run_Speed = 0.2;

    //Intake Setpoints
    public static final double Intake_Zero_Setpoint = 0.0;
    public static final double Intake_Ground_Deploy_Setpoint = 8.5;

    //====================End Effector====================
    //Conversion
    public static final double End_Effector_Absolute_To_Integrated = 0.0;

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
    public static final double End_Effector_Roller_Current_Limit = 120.0;

    //End Effector Speeds
    public static final double End_Effector_Score_Speed = 0.2;
    public static final double End_Effector_Ground_Intake_Speed = 0.2;
    public static final double End_Effector_Ground_Outake_Speed = -0.25;
    public static final double End_Effector_Coral_Station_Intake_Speed = 0.15;

    //End Effector Setpoints
    public static final double End_Effector_Wrist_Zero_Setpoint = 0.0;
    public static final double End_Effector_Wrist_L1_Score_Setpoint = 3.0;
    public static final double End_Effector_Wrist_L2_L3_Score_Setpoint = 3.25;
    public static final double End_Effector_Wrist_L4_Score_Setpoint = 3.5;

    public static final double End_Effector_Wrist_Coral_Station_Setpoint = 0.02;
    public static final double End_Effector_Wrist_Algae_Remove_Setpoint = 8.6;
    public static final double End_Effector_Wrist_Processor_Score_Setpoint = 19.0;
    public static final double End_Effector_Wrist_Barge_Score_Setpoint = 14.0;

    //====================Elevator====================
    //Elevator Motion Magic
    public static final double Elevator_kS = 0.0;
    public static final double Elevator_kV = 0.12;
    public static final double Elevator_kA = 0.05;
    public static final double Elevator_kP = 20.0; //20
    public static final double Elevator_kI = 0.0;
    public static final double Elevator_kD = 0.0;
    public static final double Elevator_Velocity = 64.0; //64
    public static final double Elevator_Acceleration = 128.0; //128
    public static final double Elevator_Jerk = 256.0; //256

    //Elevator Curent Limit
    public static final double Elevator_Current_Limit = 100.0;

    //Elevator Setpoints
    public static final double Elevator_Zero_Setpoint = 0.0;
    public static final double Elevator_Coral_Station_Setpoint = 6.5;

    public static final double Elevator_L1_Setpoint = 3.5;
    public static final double Elevator_L2_Setpoint = 6.15;
    public static final double Elevator_L3_Setpoint = 13.8;
    public static final double Elevator_L4_Setpoint = 25.3; //25.05

    public static final double Elevator_Bottom_Algae_Setpoint = 5.75;
    public static final double Elevator_Top_Algae_Setpoint = 20.5;
    public static final double Elevator_Processor_Score_Setpoint = 0.0;
    public static final double Elevator_Barge_Score_Setpoint = 26.0; 

    //====================Climb====================
    //Climb Current Limit
    public static final double Climb_Current_Limit = 120.0;

    //Climb Speeds
    public static final double Climb_Up_Speed = 0.8;
    public static final double Climb_Down_Speed = -0.8;
}