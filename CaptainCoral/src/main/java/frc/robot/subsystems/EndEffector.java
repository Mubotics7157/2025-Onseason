package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DeviceConstants;
import frc.robot.KinematicsConstants;

public class EndEffector extends SubsystemBase {
    private final TalonFX End_Effector_Wrist_Master_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_WRIST_MASTER_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Wrist_Slave_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_WRIST_SLAVE_MOTOR_DEVICE_ID);

    private final TalonFX End_Effector_Top_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_TOP_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Bottom_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_BOTTOM_MOTOR_DEVICE_ID);

    private double setpoint;

    public static EndEffector getInstance() {
        return instance;
    }

    private static EndEffector instance = new EndEffector();

    public EndEffector() {
        System.out.println("====================EndEffector Subsystem Initialized====================");

        //====================End Effector Wrist====================
        var endEffectorWristMotorConfigs = new TalonFXConfiguration();

        End_Effector_Wrist_Master_Motor.setPosition(0.0);
        End_Effector_Wrist_Slave_Motor.setPosition(0.0);

        //Brake Mode
        endEffectorWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0;
        generalSlotConfigs.kV = 0.12;
        generalSlotConfigs.kA = 0.05;
        generalSlotConfigs.kP = 10.0;
        generalSlotConfigs.kI = 0.0;
        generalSlotConfigs.kD = 0.0;

        //Motion Magic
        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 32;
        motionMagicConfigs.MotionMagicAcceleration = 64;
        motionMagicConfigs.MotionMagicJerk = 128;

        //Current Limits
        var endEffectorWristLimitConfigs = endEffectorWristMotorConfigs.CurrentLimits;
        endEffectorWristLimitConfigs.StatorCurrentLimit = 80; //120, 80
        endEffectorWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        End_Effector_Wrist_Master_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);
        End_Effector_Wrist_Slave_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);

        //====================End Effector Rollers====================
        var endEffectorRollersMotorConfigs = new TalonFXConfiguration();

        //Current Limits
        var endEffectorRollerLimitConfigs = endEffectorRollersMotorConfigs.CurrentLimits;
        endEffectorRollerLimitConfigs.StatorCurrentLimit = 30; //120, 80
        endEffectorRollerLimitConfigs.StatorCurrentLimitEnable = true;    

        //Applies Configs
        End_Effector_Top_Motor.getConfigurator().apply(endEffectorRollerLimitConfigs);
        End_Effector_Bottom_Motor.getConfigurator().apply(endEffectorRollerLimitConfigs);
    }

    public void setEndEffectorSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToEndEffectorSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        End_Effector_Wrist_Master_Motor.setControl(m_request.withPosition(this.setpoint));
        End_Effector_Wrist_Slave_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("End Effector Wrist Master Encoder", getEndEffectorWristMasterEncoder());
        SmartDashboard.putNumber("End Effector Wrist Slave Encoder", getEndEffectorWristSlaveEncoder());
    }
    
    //====================End Effector Wrist Methods====================
    public double getEndEffectorWristMasterEncoder() {
        return End_Effector_Wrist_Master_Motor.getPosition().getValueAsDouble();
    }

    public double getEndEffectorWristSlaveEncoder() {
        return End_Effector_Wrist_Slave_Motor.getPosition().getValueAsDouble();
    }

    //====================End Effector Roller Methods====================
    public void setEndEffectorRollerMotorSpeed(double speed) {
        End_Effector_Top_Motor.set(-1 * speed);
        End_Effector_Bottom_Motor.set(speed);
    }
}