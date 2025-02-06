package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        //====================End Effector Wrist Motion Magic====================
        End_Effector_Wrist_Master_Motor.setNeutralMode(NeutralModeValue.Brake);
        End_Effector_Wrist_Slave_Motor.setNeutralMode(NeutralModeValue.Brake);

        End_Effector_Wrist_Master_Motor.setPosition(0.0);
        End_Effector_Wrist_Slave_Motor.setPosition(0.0);

        var endEffectorWristMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0;
        generalSlotConfigs.kV = 0.12;
        generalSlotConfigs.kA = 0.05;
        generalSlotConfigs.kP = 2.5;
        generalSlotConfigs.kI = 0.0;
        generalSlotConfigs.kD = 0.0;

        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 16;
        motionMagicConfigs.MotionMagicAcceleration = 32;
        motionMagicConfigs.MotionMagicJerk = 64;

        End_Effector_Wrist_Master_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);
        End_Effector_Wrist_Slave_Motor.getConfigurator().apply(endEffectorWristMotorConfigs); //POINT OF ERROR

        //====================End Effector Wrist Master Current Limit====================
        var endEffectorWristMasterConfigurator = End_Effector_Wrist_Master_Motor.getConfigurator();
        var endEffectorMasterLimitConfigs = new CurrentLimitsConfigs();

        endEffectorMasterLimitConfigs.StatorCurrentLimit = 120;
        endEffectorMasterLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorWristMasterConfigurator.apply(endEffectorMasterLimitConfigs);

        //====================End Effector Wrist Slave Current Limit====================
        var endEffectorWristSlaveConfigurator = End_Effector_Wrist_Slave_Motor.getConfigurator();
        var endEffectorSlaveLimitConfigs = new CurrentLimitsConfigs();

        endEffectorSlaveLimitConfigs.StatorCurrentLimit = 120;
        endEffectorSlaveLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorWristSlaveConfigurator.apply(endEffectorSlaveLimitConfigs);

        // //====================End Effector Top Roller Current Limit====================
        var endEffectorTopConfigurator = End_Effector_Top_Motor.getConfigurator();
        var endEffectorTopLimitConfigs = new CurrentLimitsConfigs();

        endEffectorTopLimitConfigs.StatorCurrentLimit = 120;
        endEffectorTopLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorTopConfigurator.apply(endEffectorTopLimitConfigs);

        //====================End Effector Bottom Roller Current Limit====================
        var endEffectorBottomConfigurator = End_Effector_Bottom_Motor.getConfigurator();
        var endEffectorBottomLimitConfigs = new CurrentLimitsConfigs();

        endEffectorBottomLimitConfigs.StatorCurrentLimit = 120;
        endEffectorBottomLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorBottomConfigurator.apply(endEffectorBottomLimitConfigs);
    }

    public void setEndEffectorSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToEndEffectorSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        End_Effector_Wrist_Master_Motor.setControl(m_request.withPosition(this.setpoint));
        End_Effector_Wrist_Slave_Motor.setControl(m_request.withPosition(-1 * this.setpoint)); //POINT OF ERROR
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