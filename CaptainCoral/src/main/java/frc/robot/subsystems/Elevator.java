package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        System.out.println("====================Elevator Subsystem Online====================");

        //====================Elevator Subsystem====================
        var elevatorMotorConfigs = new TalonFXConfiguration();

        Elevator_Master_Motor.setPosition(KinematicsConstants.Absolute_Zero);
        Elevator_Slave_Motor.setPosition(KinematicsConstants.Absolute_Zero);

        //Brake Mode
        elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = elevatorMotorConfigs.Slot0;
        generalSlotConfigs.kS = KinematicsConstants.Elevator_kS;
        generalSlotConfigs.kV = KinematicsConstants.Elevator_kV;
        generalSlotConfigs.kA = KinematicsConstants.Elevator_kV;
        generalSlotConfigs.kP = KinematicsConstants.Elevator_kP;
        generalSlotConfigs.kI = KinematicsConstants.Elevator_kI;
        generalSlotConfigs.kD = KinematicsConstants.Elevator_kD;

        //Motion Magic
        var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = KinematicsConstants.Elevator_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = KinematicsConstants.Elevator_Acceleration;
        motionMagicConfigs.MotionMagicJerk = KinematicsConstants.Elevator_Jerk;

        //Current limits
        var limitConfigs = elevatorMotorConfigs.CurrentLimits;
        limitConfigs.StatorCurrentLimit = KinematicsConstants.Elevator_Current_Limit;
        limitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Elevator_Master_Motor.getConfigurator().apply(elevatorMotorConfigs);
        Elevator_Slave_Motor.getConfigurator().apply(elevatorMotorConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Master Encoder", getElevatorMasterEncoder());
        SmartDashboard.putNumber("Elevator Slave Encoder", getElevatorSlaveEncoder());
    }

    //====================Elevator Methods====================
    public void setElevatorMotorSpeed(double speed) {
        Elevator_Master_Motor.set(speed);
        Elevator_Slave_Motor.set(speed);
    }

    public double getElevatorMasterEncoder() {
        return Elevator_Master_Motor.getPosition().getValueAsDouble();
    }
 
    public double getElevatorSlaveEncoder() {
        return Elevator_Slave_Motor.getPosition().getValueAsDouble();
    }

    public void setElevatorSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToElevatorSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.Absolute_Zero);
        Elevator_Master_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
        Elevator_Slave_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
    }
}