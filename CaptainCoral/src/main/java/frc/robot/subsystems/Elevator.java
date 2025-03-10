package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Devices;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX Elevator_Master_Motor = new TalonFX(Devices.ELEVATOR_MASTER_MOTOR);
    private final TalonFX Elevator_Slave_Motor = new TalonFX(Devices.ELEVATOR_SLAVE_MOTOR);

    private double setpoint;

    public static Elevator getInstance() {
        return instance;
    }

    private static Elevator instance = new Elevator();

    public Elevator() {
        System.out.println("====================Elevator Subsystem Online====================");

        //PUT NUMBER HERE

        //====================Elevator Subsystem====================
        var elevatorMotorConfigs = new TalonFXConfiguration();

        Elevator_Master_Motor.setPosition(Constants.Absolute_Zero);
        Elevator_Slave_Motor.setPosition(Constants.Absolute_Zero);

        //Brake Mode
        elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = elevatorMotorConfigs.Slot0;
        generalSlotConfigs.kG = Constants.Elevator_kG;
        generalSlotConfigs.kS = Constants.Elevator_kS;
        generalSlotConfigs.kV = Constants.Elevator_kV;
        generalSlotConfigs.kA = Constants.Elevator_kV;
        generalSlotConfigs.kP = Constants.Elevator_kP;
        generalSlotConfigs.kI = Constants.Elevator_kI;
        generalSlotConfigs.kD = Constants.Elevator_kD;

        // //Motion Magic
        var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator_Velocity;
        // motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator_Acceleration;

        //Current limits
        var limitConfigs = elevatorMotorConfigs.CurrentLimits;
        limitConfigs.StatorCurrentLimit = Constants.Elevator_Current_Limit;
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
        final MotionMagicVoltage m_request = new MotionMagicVoltage(Constants.Absolute_Zero);
        Elevator_Master_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
        Elevator_Slave_Motor.setControl(m_request.withPosition(-1 * this.setpoint));
    }

    public void zeroElevator() {
        Elevator_Master_Motor.setPosition(Constants.Absolute_Zero);
        Elevator_Slave_Motor.setPosition(Constants.Absolute_Zero);
    }

    //make method called reconfigure pid, copy and paste pid settings 

    // var elevatorMotorConfigs = new TalonFXConfiguration();

    //     Elevator_Master_Motor.setPosition(Constants.Absolute_Zero);
    //     Elevator_Slave_Motor.setPosition(Constants.Absolute_Zero);

    //     //Brake Mode
    //     elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //     //General Configurations
    //     var generalSlotConfigs = elevatorMotorConfigs.Slot0;
    //     generalSlotConfigs.kS = Constants.Elevator_kS; //GET
    //     generalSlotConfigs.kV = Constants.Elevator_kV; //GET
    //     generalSlotConfigs.kA = Constants.Elevator_kV; //GET
    //     generalSlotConfigs.kP = Constants.Elevator_kP; //GET
    //     generalSlotConfigs.kI = Constants.Elevator_kI; //GET
    //     generalSlotConfigs.kD = Constants.Elevator_kD; //GET

    //     //Motion Magic
    //     var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
    //     motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator_Velocity;
    //     motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator_Acceleration;
    //     motionMagicConfigs.MotionMagicJerk = Constants.Elevator_Jerk;

    //     //Current limits
    //     var limitConfigs = elevatorMotorConfigs.CurrentLimits;
    //     limitConfigs.StatorCurrentLimit = Constants.Elevator_Current_Limit;
    //     limitConfigs.StatorCurrentLimitEnable = true;

    //     //Applies Configs
    //     Elevator_Master_Motor.getConfigurator().apply(elevatorMotorConfigs);
    //     Elevator_Slave_Motor.getConfigurator().apply(elevatorMotorConfigs);
}