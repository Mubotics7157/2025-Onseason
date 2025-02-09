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

public class Intake extends SubsystemBase {
    private final TalonFX Intake_Wrist_Motor = new TalonFX(DeviceConstants.INTAKE_WRIST_MOTOR_DEVICE_ID);

    private final TalonFX Intake_Roller_Motor = new TalonFX(DeviceConstants.INTAKE_ROLLER_MOTOR_DEVICE_ID);

    private final TalonFX Intake_Indexer_Master_Motor = new TalonFX(DeviceConstants.INTAKE_INDEXER_MASTER_MOTOR_DEVICE_ID);
    private final TalonFX Intake_Indexer_Slave_Motor = new TalonFX(DeviceConstants.INTAKE_INDEXER_MASTER_MOTOR_DEVICE_ID);

    private double setpoint;
    
    public static Intake getInstance() {
        return instance;
    }

    private static Intake instance = new Intake();

    public Intake() {
        System.out.println("====================Intake Subsystem Initialized====================");

        //====================Intake Wrist====================
        var intakeWristMotorConfigs = new TalonFXConfiguration();

        Intake_Wrist_Motor.setPosition(0.0);

        //Brake Mode
        intakeWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = intakeWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0;
        generalSlotConfigs.kV = 0.12;
        generalSlotConfigs.kA = 0.05;
        generalSlotConfigs.kP = 10.0;
        generalSlotConfigs.kI = 0.0;
        generalSlotConfigs.kD = 0.0;

        //Motion Magic
        var motionMagicConfigs = intakeWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 32;
        motionMagicConfigs.MotionMagicAcceleration = 64;
        motionMagicConfigs.MotionMagicJerk = 128;

        //Current Limits
        var intakeWristLimitConfigs = intakeWristMotorConfigs.CurrentLimits;
        intakeWristLimitConfigs.StatorCurrentLimit = 80; //120, 80
        intakeWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Wrist_Motor.getConfigurator().apply(intakeWristMotorConfigs);

        //====================Intake Rollers====================
        var intakeRollersMotorConfigs = new TalonFXConfiguration();

        //Current Limits
        var intakeRollerLimitConfigs = intakeRollersMotorConfigs.CurrentLimits;
        intakeRollerLimitConfigs.StatorCurrentLimit = 30; //120, 80
        intakeRollerLimitConfigs.StatorCurrentLimitEnable = true;    

        //Applies Configs
        Intake_Roller_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
        Intake_Indexer_Master_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
        Intake_Indexer_Slave_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
    }

    public void setIntakeSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToIntakeSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        Intake_Wrist_Motor.setControl(m_request.withPosition(this.setpoint));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Wrist Encoder", getIntakeWristEncoder());
    }
    
    //====================Intake Wrist Methods====================
    public double getIntakeWristEncoder() {
        return Intake_Wrist_Motor.getPosition().getValueAsDouble();
    }

    //====================Intake Roller Methods====================
    public void setIntakeRollerMotorSpeed(double speed) {
        Intake_Roller_Motor.set(speed);
    }

    //====================Indexer Methods====================
    public void setIndexerMotorSpeed(double speed) {
        Intake_Indexer_Master_Motor.set(speed);
        Intake_Indexer_Slave_Motor.set(speed);
    }
}