package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DeviceConstants;
import frc.robot.KinematicsConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX Elevator_Master_Motor = new TalonFX(DeviceConstants.ELEVATOR_MASTER_MOTOR_DEVICE_ID);
    private final TalonFX Elevator_Slave_Motor = new TalonFX(DeviceConstants.ELEVATOR_SLAVE_MOTOR_DEVICE_ID);

    private double setpoint;

    public static Elevator getInstance() {
        return instance;
    }

    private static Elevator instance = new Elevator();

    public Elevator() {
        //====================Elevator Motion Magic====================
        Elevator_Master_Motor.setNeutralMode(NeutralModeValue.Brake);
        Elevator_Slave_Motor.setNeutralMode(NeutralModeValue.Brake);
        System.out.println("Brake Mode Enabled");

        Elevator_Master_Motor.setPosition(0.0);
        Elevator_Slave_Motor.setPosition(0.0);

        var elevatorMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = elevatorMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0;
        generalSlotConfigs.kV = 0.12;
        generalSlotConfigs.kA = 0.05;
        generalSlotConfigs.kP = 6.0;
        generalSlotConfigs.kI = 0.0;
        generalSlotConfigs.kD = 0.0;

        var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 16;
        motionMagicConfigs.MotionMagicAcceleration = 32;
        motionMagicConfigs.MotionMagicJerk = 64;

        Elevator_Master_Motor.getConfigurator().apply(elevatorMotorConfigs);
        Elevator_Slave_Motor.getConfigurator().apply(elevatorMotorConfigs);

        //====================Elevator Master Current Limit====================
        var elevatorMasterConfigurator = Elevator_Master_Motor.getConfigurator();
        var elevatorMasterLimitConfigs = new CurrentLimitsConfigs();

        elevatorMasterLimitConfigs.StatorCurrentLimit = 120;
        elevatorMasterLimitConfigs.StatorCurrentLimitEnable = true;
        elevatorMasterConfigurator.apply(elevatorMasterLimitConfigs);

        //====================Elevator Slave Current Limit====================
        var elevatorSlaveConfigurator = Elevator_Slave_Motor.getConfigurator();
        var elevatorSlaveLimitConfigs = new CurrentLimitsConfigs();

        elevatorSlaveLimitConfigs.StatorCurrentLimit = 120;
        elevatorSlaveLimitConfigs.StatorCurrentLimitEnable = true;
        elevatorSlaveConfigurator.apply(elevatorSlaveLimitConfigs);
    }

    public void setElevatorSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToElevatorSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        Elevator_Master_Motor.setControl(m_request.withPosition(this.setpoint));
        Elevator_Slave_Motor.setControl(m_request.withPosition(this.setpoint));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Master Encoder", getElevatorMasterEncoder());
        SmartDashboard.putNumber("Elevator Slave Encoder", getElevatorSlaveEncoder());
    }

    //====================Elevator Methods====================
    public double getElevatorMasterEncoder() {
        return Elevator_Master_Motor.getPosition().getValueAsDouble();
    }

    public double getElevatorSlaveEncoder() {
        return Elevator_Slave_Motor.getPosition().getValueAsDouble();
    }

    public void setElevatorMotorSpeed(double speed) {
        Elevator_Master_Motor.set(speed);
        Elevator_Slave_Motor.set(speed);
    }
}