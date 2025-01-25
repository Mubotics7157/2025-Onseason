package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.DeviceConstants;

public class Intake extends SubsystemBase {
    private final TalonFX Intake_Wrist_Motor = new TalonFX(DeviceConstants.INTAKE_WRIST_MOTOR_DEVICE_ID);

    private final TalonFX Intake_Roller_Motor = new TalonFX(DeviceConstants.INTAKE_ROLLER_MOTOR_DEVICE_ID);
    private final TalonFX Intake_Indexer_Motor = new TalonFX(DeviceConstants.INTAKE_INDEXER_MOTOR_DEVICE_ID);
    
    public static Intake getInstance() {
        return instance;
    }

    private static Intake instance = new Intake();

    public Intake() {
        //====================Elevator Motion Magic====================
        var intakeWristMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = intakeWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
        generalSlotConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        generalSlotConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        generalSlotConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        generalSlotConfigs.kI = 0; // no output for integrated error
        generalSlotConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = intakeWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        Intake_Wrist_Motor.getConfigurator().apply(motionMagicConfigs);

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
        var intakeIndexerConfigurator = Intake_Indexer_Motor.getConfigurator();
        var intakeIndexerLimitConfigs = new CurrentLimitsConfigs();

        intakeIndexerLimitConfigs.StatorCurrentLimit = 120;
        intakeIndexerLimitConfigs.StatorCurrentLimitEnable = true;
        intakeIndexerConfigurator.apply(intakeIndexerLimitConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Wrist Encoder", getIntakeWristEncoder());
    }
    
    //====================Intake Wrist Methods====================
    public double getIntakeWristEncoder() {
        return Intake_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public void setIntakeWristMotorSpeed(double speed) {
        Intake_Wrist_Motor.set(speed);
    }

    //====================Intake Roller Methods====================
    public void setIntakeRollerMotorSpeed(double speed) {
        Intake_Roller_Motor.set(speed);
    }

    //====================Indexer Methods====================
    public void setIndexerMotorSpeed(double speed) {
        Intake_Indexer_Motor.set(speed);
    }
}