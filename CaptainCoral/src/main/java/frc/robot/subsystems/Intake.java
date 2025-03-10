package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Devices;
import frc.robot.Utility.AbsoluteEncoder;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX Intake_Wrist_Motor = new TalonFX(Devices.INTAKE_WRIST_MOTOR);
    private final TalonFX Intake_Roller_Motor = new TalonFX(Devices.INTAKE_ROLLER_MOTOR);
    private final TalonFX Intake_Indexer_Master_Motor = new TalonFX(Devices.INDEXER_MOTOR);

    private final AbsoluteEncoder encoder = new AbsoluteEncoder(7);

    private double setpoint;
    
    public static Intake getInstance() {
        return instance;
    }

    private static Intake instance = new Intake();

    public Intake() {
        System.out.println("====================Intake Subsystem Online====================");

        //====================Intake Wrist====================
        var intakeWristMotorConfigs = new TalonFXConfiguration();

        encoder.setInverted(true);

        encoder.setOffset(Rotation2d.fromRotations(0.94312122289));

        //Brake Mode
        intakeWristMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; //SET ME TO BRAKE

        //General Configurations
        var generalSlotConfigs = intakeWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = Constants.Intake_Wrist_kS;
        generalSlotConfigs.kV = Constants.Intake_Wrist_kV;
        generalSlotConfigs.kA = Constants.Intake_Wrist_kA;
        generalSlotConfigs.kP = Constants.Intake_Wrist_kP;
        generalSlotConfigs.kI = Constants.Intake_Wrist_kI;
        generalSlotConfigs.kD = Constants.Intake_Wrist_kD;

        //Motion Magic
        var motionMagicConfigs = intakeWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake_Wrist_Velocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Intake_Wrist_Acceleration;

        //Current Limits
        var intakeWristLimitConfigs = intakeWristMotorConfigs.CurrentLimits;
        intakeWristLimitConfigs.StatorCurrentLimit = Constants.Intake_Wrist_Current_Limit;
        intakeWristLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Wrist_Motor.getConfigurator().apply(intakeWristMotorConfigs);

        //====================Intake Rollers====================
        var intakeRollersMotorConfigs = new TalonFXConfiguration();

        //Current Limits
        var intakeRollerLimitConfigs = intakeRollersMotorConfigs.CurrentLimits;
        intakeRollerLimitConfigs.StatorCurrentLimit = Constants.Intake_Roller_Current_Limit;
        intakeRollerLimitConfigs.StatorCurrentLimitEnable = true;

        //Applies Configs
        Intake_Roller_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
        Intake_Indexer_Master_Motor.getConfigurator().apply(intakeRollersMotorConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Wrist Encoder", getIntakeWristEncoder());
        SmartDashboard.putNumber("Intake Through Bore Rotations", getIntakeWristThroughBoreWithOffset());
    }
    
    //====================Intake Wrist Methods====================
    public void zeroIntakeWrist() {
        Intake_Wrist_Motor.setPosition(Constants.Absolute_Zero);
    }

    public double getIntakeWristEncoder() {
        return Intake_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public void zeroIntakeWristWithAbsolute() {
        Intake_Wrist_Motor.setPosition(getIntakeWristThroughBoreWithOffset());
    }

    public double getIntakeWristThroughBoreWithOffset() {
        return encoder.getPosition().getRotations() * 28.13;
    }

    public void setIntakeWristSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToIntakeWristSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(Constants.Absolute_Zero);
        Intake_Wrist_Motor.setControl(m_request.withPosition(this.setpoint));
    }

    public void setIntakeWristSpeed(double speed) {
        Intake_Wrist_Motor.set(speed);
    }

    //====================Intake Roller Methods====================
    public void setIntakeRollerMotorSpeed(double speed) {
        Intake_Roller_Motor.set(speed);
    }

    //====================Indexer Methods====================
    public void setIndexerMotorSpeed(double speed) {
        Intake_Indexer_Master_Motor.set(-0.7 * speed);
    }
}