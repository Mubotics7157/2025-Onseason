package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DeviceConstants;
import frc.robot.PhysConstants;

public class Intake extends SubsystemBase {
    private final TalonFX Intake_Wrist_Motor = new TalonFX(DeviceConstants.INTAKE_WRIST_MOTOR_DEVICE_ID);
    private final TalonFX Intake_Roller_Motor = new TalonFX(DeviceConstants.INTAKE_ROLLER_MOTOR_DEVICE_ID);
    private final TalonFX Intake_Indexer_Master_Motor = new TalonFX(DeviceConstants.INDEXER_MOTOR_DEVICE_ID);

    private final DutyCycleEncoder Intake_Wrist_Through_Bore = new DutyCycleEncoder(new DigitalInput(DeviceConstants.INTAKE_WRIST_THROUGH_BORE_PORT));
    //private final DigitalInput Indexer_Sensor = new DigitalInput(DeviceConstants.INDEXER_PHOTOELECTRIC_PORT);

    private double setpoint;
    
    public static Intake getInstance() {
        return instance;
    }

    private static Intake instance = new Intake();

    public Intake() {
        System.out.println("====================Intake Subsystem Online====================");

        //====================Intake Wrist====================
        var intakeWristMotorConfigs = new TalonFXConfiguration();

        Intake_Wrist_Motor.setPosition(PhysConstants.Absolute_Zero);

        //Brake Mode
        intakeWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //General Configurations
        var generalSlotConfigs = intakeWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = PhysConstants.Intake_Wrist_kS;
        generalSlotConfigs.kV = PhysConstants.Intake_Wrist_kV;
        generalSlotConfigs.kA = PhysConstants.Intake_Wrist_kA;
        generalSlotConfigs.kP = PhysConstants.Intake_Wrist_kP;
        generalSlotConfigs.kI = PhysConstants.Intake_Wrist_kI;
        generalSlotConfigs.kD = PhysConstants.Intake_Wrist_kD;

        //Motion Magic
        var motionMagicConfigs = intakeWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PhysConstants.Intake_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = PhysConstants.Intake_Wrist_Acceleration;
        motionMagicConfigs.MotionMagicJerk = PhysConstants.Intake_Wrist_Jerk;

        //Current Limits
        var intakeWristLimitConfigs = intakeWristMotorConfigs.CurrentLimits;
        intakeWristLimitConfigs.StatorCurrentLimit = PhysConstants.Intake_Wrist_Current_Limit;
        intakeWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Wrist_Motor.getConfigurator().apply(intakeWristMotorConfigs);

        //====================Intake Rollers====================
        var intakeRollersMotorConfigs = new TalonFXConfiguration();

        //Current Limits
        var intakeRollerLimitConfigs = intakeRollersMotorConfigs.CurrentLimits;
        intakeRollerLimitConfigs.StatorCurrentLimit = PhysConstants.Intake_Roller_Current_Limit;
        intakeRollerLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Roller_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
        Intake_Indexer_Master_Motor.getConfigurator().apply(intakeRollersMotorConfigs);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Wrist Encoder", getIntakeWristEncoder());
        SmartDashboard.putNumber("Intake Through Bore Encoder", getIntakeWristThroughBoreWithOffset());
        //SmartDashboard.putBoolean("Indexer Sensor Reading", getIndexerSensorReading());
    }
    
    //====================Intake Wrist Methods====================
    public double getIntakeWristEncoder() {
        return Intake_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public double getIntakeWristThroughBoreWithOffset() {
        return Intake_Wrist_Through_Bore.get() - 0.4;
    }

    public void setIntakeWristSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToIntakeWristSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(PhysConstants.Absolute_Zero);
        Intake_Wrist_Motor.setControl(m_request.withPosition(this.setpoint));
    }

    public void zeroIntakeWrist() {
        Intake_Wrist_Motor.setPosition(getIntakeWristThroughBoreWithOffset()); //MAKE SURE UNITS MATCH
    }

    //====================Intake Roller Methods====================
    public void setIntakeRollerMotorSpeed(double speed) {
        Intake_Roller_Motor.set(speed);
    }

    //====================Indexer Methods====================
    public void setIndexerMotorSpeed(double speed) {
        Intake_Indexer_Master_Motor.set(-1 * 0.7 * speed);
    }

    // public boolean getIndexerSensorReading() {
    //     return !Indexer_Sensor.get();
    // }
}