package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DeviceConstants;
import frc.robot.PhysConstants;

public class EndEffector extends SubsystemBase {
    private final TalonFX End_Effector_Wrist_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_WRIST_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Top_Roller_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_TOP_ROLLER_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Bottom_Roller_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_BOTTOM_ROLLER_MOTOR_DEVICE_ID);

    private final DutyCycleEncoder End_Effector_Wrist_Through_Bore_Encoder = new DutyCycleEncoder(new DigitalInput(DeviceConstants.END_EFFECTOR_WRIST_THROUGH_BORE_PORT));
    private final DigitalInput End_Effector_Sensor = new DigitalInput(DeviceConstants.END_EFFECTOR_PHOTOELECTRIC_PORT);

    private double setpoint;

    public static EndEffector getInstance() {
        return instance;
    }

    private static EndEffector instance = new EndEffector();

    public EndEffector() {
        System.out.println("====================EndEffector Subsystem Online====================");

        //====================End Effector Wrist====================
        var endEffectorWristMotorConfigs = new TalonFXConfiguration();

        End_Effector_Wrist_Motor.setPosition(getEndEffectorWristThroughBore() * PhysConstants.End_Effector_Absolute_To_Integrated); //Change to KinematicsConstants.Absolute_Zero if not working

        //Brake Mode
        endEffectorWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = PhysConstants.End_Effector_Wrist_kS;
        generalSlotConfigs.kV = PhysConstants.End_Effector_Wrist_kV;
        generalSlotConfigs.kA = PhysConstants.End_Effector_Wrist_kA;
        generalSlotConfigs.kP = PhysConstants.End_Effector_Wrist_kP;
        generalSlotConfigs.kI = PhysConstants.End_Effector_Wrist_kI;
        generalSlotConfigs.kD = PhysConstants.End_Effector_Wrist_kD;

        //Motion Magic
        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PhysConstants.End_Effector_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = PhysConstants.End_Effector_Wrist_Acceleration;
        motionMagicConfigs.MotionMagicJerk = PhysConstants.End_Effector_Wrist_Jerk;

        //Current Limits
        var endEffectorWristLimitConfigs = endEffectorWristMotorConfigs.CurrentLimits;
        endEffectorWristLimitConfigs.StatorCurrentLimit = PhysConstants.End_Effector_Wrist_Current_Limit;
        endEffectorWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        End_Effector_Wrist_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);

        //====================End Effector Rollers====================
        var endEffectorRollerMotorsConfigs = new TalonFXConfiguration();

        //Current Limits
        var endEffectorRollerLimitConfigs = endEffectorRollerMotorsConfigs.CurrentLimits;
        endEffectorRollerLimitConfigs.StatorCurrentLimit = PhysConstants.End_Effector_Roller_Current_Limit;
        endEffectorRollerLimitConfigs.StatorCurrentLimitEnable = true; 

        //Applies Configs
        End_Effector_Top_Roller_Motor.getConfigurator().apply(endEffectorRollerLimitConfigs);
        End_Effector_Bottom_Roller_Motor.getConfigurator().apply(endEffectorRollerLimitConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("End Effector Wrist Master Encoder", getEndEffectorWristEncoder());
        SmartDashboard.putNumber("End Effector Through Bore Encoder", getEndEffectorWristThroughBore());
        SmartDashboard.putBoolean("End Effector Sensor Reading", getEndEffectorSensorReading());
    }
    
    //====================End Effector Wrist Methods====================
    public double getEndEffectorWristEncoder() {
        return End_Effector_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public double getEndEffectorWristThroughBore() {
        return End_Effector_Wrist_Through_Bore_Encoder.get();
    }

    public void setEndEffectorWristSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToEndEffectorWristSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(PhysConstants.Absolute_Zero);
        End_Effector_Wrist_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
    }

    //====================End Effector Roller Methods====================
    public void setEndEffectorRollerMotorSpeed(double speed) {
        End_Effector_Top_Roller_Motor.set(-1 * speed);
        End_Effector_Bottom_Roller_Motor.set(speed);
    }

    public boolean getEndEffectorSensorReading() {
        return !End_Effector_Sensor.get();
    }
}