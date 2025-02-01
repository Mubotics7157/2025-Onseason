package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
        //====================Intake Wrist Motion Magic====================
        Intake_Wrist_Motor.setPosition(0.0);

        var intakeWristMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = intakeWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0;
        generalSlotConfigs.kV = 0.12;
        generalSlotConfigs.kA = 0.05;
        generalSlotConfigs.kP = 0.001;
        generalSlotConfigs.kI = 0;
        generalSlotConfigs.kD = 0.0;

        var motionMagicConfigs = intakeWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 16;
        motionMagicConfigs.MotionMagicAcceleration = 32;
        motionMagicConfigs.MotionMagicJerk = 64;

        Intake_Wrist_Motor.getConfigurator().apply(intakeWristMotorConfigs);

        //====================Intake Wrist Current Limit====================
        var intakeWristConfigurator = Intake_Wrist_Motor.getConfigurator();
        var intakeWristLimitConfigs = new CurrentLimitsConfigs();

        intakeWristLimitConfigs.StatorCurrentLimit = 120;
        intakeWristLimitConfigs.StatorCurrentLimitEnable = true;
        intakeWristConfigurator.apply(intakeWristLimitConfigs);

        //====================Intake Roller Current Limit====================
        var intakeRollerConfigurator = Intake_Roller_Motor.getConfigurator();
        var intakeRollerLimitConfigs = new CurrentLimitsConfigs();

        intakeRollerLimitConfigs.StatorCurrentLimit = 120;
        intakeRollerLimitConfigs.StatorCurrentLimitEnable = true;
        intakeRollerConfigurator.apply(intakeRollerLimitConfigs);

        //====================Intake Indexer Current Limit====================
        var intakeMasterIndexerConfigurator = Intake_Indexer_Master_Motor.getConfigurator();
        var intakeMasterIndexerLimitConfigs = new CurrentLimitsConfigs();

        intakeMasterIndexerLimitConfigs.StatorCurrentLimit = 120;
        intakeMasterIndexerLimitConfigs.StatorCurrentLimitEnable = true;
        intakeMasterIndexerConfigurator.apply(intakeMasterIndexerLimitConfigs);

        //====================Intake Indexer Current Limit====================
        var intakeSlaveIndexerConfigurator = Intake_Indexer_Slave_Motor.getConfigurator();
        var intakeSlaveIndexerLimitConfigs = new CurrentLimitsConfigs();

        intakeSlaveIndexerLimitConfigs.StatorCurrentLimit = 120;
        intakeSlaveIndexerLimitConfigs.StatorCurrentLimitEnable = true;
        intakeSlaveIndexerConfigurator.apply(intakeSlaveIndexerLimitConfigs);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    private void goToSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        Intake_Wrist_Motor.setControl(m_request.withPosition(this.setpoint));
    }

    @Override
    public void periodic() {
        goToSetpoint();
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