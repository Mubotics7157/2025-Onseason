package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static final double Drivetrain_X_Pose_kP = 0.001;
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
    //Intake Motion Magic
    public static final double Intake_Wrist_kS = 0.0;
    public static final double Intake_Wrist_kV = 0.12;
    public static final double Intake_Wrist_kA = 0.05;
    public static final double Intake_Wrist_kP = 0.75;
    public static final double Intake_Wrist_kI = 0.0;
    public static final double Intake_Wrist_kD = 0.0;
    public static final double Intake_Wrist_Velocity = 2048.0;
    public static final double Intake_Wrist_Acceleration = 4096.0;

    //Intake Current Limits
    public static final double Intake_Wrist_Current_Limit = 100.0;
    public static final double Intake_Roller_Current_Limit = 100.0;

    //Intake Speeds
    public static final double Intake_Ground_Run_Speed = -0.75;
    public static final double Outake_Ground_Run_Speed = 0.2;

    //Intake Setpoints
    public static final double Intake_Zero_Setpoint = 3.0;
    public static final double Intake_Ground_Deploy_Setpoint = 14.25;

    //====================End Effector====================
    //Conversion
    public static final double End_Effector_Absolute_To_Integrated = 0.0;

    //End Effector Motion Magic
    public static final double End_Effector_Wrist_kS = 0.0;
    public static final double End_Effector_Wrist_kV = 0.12;
    public static final double End_Effector_Wrist_kA = 0.05;
    public static final double End_Effector_Wrist_kP = 0.75;
    public static final double End_Effector_Wrist_kI = 0.0;
    public static final double End_Effector_Wrist_kD = 0.0;
    public static final double End_Effector_Wrist_Velocity = 2048.0;
    public static final double End_Effector_Wrist_Acceleration = 4096.0;

    //End Effector Current Limits
    public static final double End_Effector_Wrist_Current_Limit = 100.0;
    public static final double End_Effector_Roller_Current_Limit = 120.0;

    //~~~~~End Effector Speeds~~~~~
    //Coral
    public static final double End_Effector_Ground_Intake_Speed = 0.55;
    public static final double End_Effector_Ground_Outake_Speed = -0.4; //-0.25
    public static final double End_Effector_Coral_Station_Intake_Speed = 0.15;
    public static final double End_Effector_Score_Coral_Speed = 0.20;

    //Algae
    public static final double End_Effector_Algae_Intake_Speed = 1.0;
    public static final double End_Effector_Algae_Score_Speed = -1.0;

    //~~~~~End Effector Setpoints~~~~~
    //Coral
    public static final double End_Effector_Wrist_Zero_Setpoint = 0.0;
    public static final double End_Effector_Wrist_Coral_Ground_Setpoint = 0.55;
    public static final double End_Effector_Wrist_L1_Score_Setpoint = 3.0;
    public static final double End_Effector_Wrist_L2_L3_Score_Setpoint = 3.25;
    public static final double End_Effector_Wrist_L4_Score_Setpoint = 3.5;
    public static final double End_Effector_Wrist_Coral_Station_Setpoint = 0.02;

    //Algae  
    public static final double End_Effector_Wrist_Algae_Remove_Setpoint = 9.25;
    public static final double End_Effector_Wrist_Algae_Ground_Setpoint = 12.0;
    public static final double End_Effector_Wrist_Net_Score_Setpoint = 3.825;

    //====================Elevator====================
    //Elevator Motion Magic
    public static final double Elevator_kG = 0.0; //CHANGE ME
    public static final double Elevator_kS = 0.0; //0.5
    public static final double Elevator_kV = 0.12; //0.12
    public static final double Elevator_kA = 0.05; //0.05
    public static final double Elevator_kP = 0.75; //0.75
    public static final double Elevator_kI = 0.0; //0.0
    public static final double Elevator_kD = 0.0; //0.0
    public static final double Elevator_Velocity = 2048.0; //2048
    public static final double Elevator_Acceleration = 4096.0; //4096

    //Elevator Curent Limit
    public static final double Elevator_Current_Limit = 80.0;

    //~~~~~Elevator Setpoints~~~~~
    //Coral
    public static final double Elevator_Zero_Setpoint = 0.0;
    public static final double Elevator_Ground_Coral_Setpoint = 0.5;
    public static final double Elevator_Coral_Station_Setpoint = 6.5;
    public static final double Elevator_L1_Setpoint = 3.5;
    public static final double Elevator_L2_Setpoint = 6.15;
    public static final double Elevator_L3_Setpoint = 13.8;
    public static final double Elevator_L4_Setpoint = 25.4;

    //Algae
    public static final double Elevator_Bottom_Algae_Setpoint = 7.25;
    public static final double Elevator_Top_Algae_Setpoint = 14.5;
    public static final double Elevator_Ground_Algae_Setpoint = 2.25;
    public static final double Elevator_Net_Score_Setpoint = 25.4;

    //====================Climb====================
    //Climb Current Limit
    public static final double Climb_Current_Limit = 120.0;

    //Climb Speeds
    public static final double Climb_Up_Speed = 1.0;
    public static final double Climb_Down_Speed = -1.0;
}