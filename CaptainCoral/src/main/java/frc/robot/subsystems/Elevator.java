package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
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
        var elevatorMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = elevatorMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0; // Add 0.25 V output to overcome static friction
        generalSlotConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        generalSlotConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        generalSlotConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        generalSlotConfigs.kI = 0; // no output for integrated error
        generalSlotConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

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

    @Override
    public void periodic() {
        goToSetpoint();
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
    }

    //====================Elevator Methods====================
    public double getElevatorEncoder() {
        return Elevator_Master_Motor.getPosition().getValueAsDouble();
    }

    public void setElevatorMotorSpeed(double speed) {
        Elevator_Master_Motor.set(speed);
        Elevator_Slave_Motor.set(speed);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    private void goToSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        Elevator_Master_Motor.setControl(m_request.withPosition(this.setpoint));
        Elevator_Slave_Motor.setControl(m_request.withPosition(this.setpoint));
    }
}
