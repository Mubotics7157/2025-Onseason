package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.DeviceConstants;

public class EndEffector extends SubsystemBase {
    private final TalonFX End_Effector_Wrist_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_WRIST_MOTOR_DEVICE_ID);

    private final TalonFX End_Effector_Top_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_TOP_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Bottom_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_BOTTOM_MOTOR_DEVICE_ID);

    private final DoubleSolenoid Claw_Solenoid_1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DeviceConstants.CLAW_SOLENOID_1_FORWARD_CHANNEL, DeviceConstants.CLAW_SOLENOID_1_REVERSE_CHANNEL);
    private final DoubleSolenoid Claw_Solenoid_2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DeviceConstants.CLAW_SOLENOID_2_FORWARD_CHANNEL, DeviceConstants.CLAW_SOLENOID_2_REVERSE_CHANNEL);

    public static EndEffector getInstance() {
        return instance;
    }

    private static EndEffector instance = new EndEffector();

    public EndEffector() {
        //====================End Effector Wrist Motion Magic====================
        var endEffectorWristMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
        generalSlotConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        generalSlotConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        generalSlotConfigs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        generalSlotConfigs.kI = 0; // no output for integrated error
        generalSlotConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        End_Effector_Wrist_Motor.getConfigurator().apply(motionMagicConfigs);

        //====================End Effector Wrist Current Limit====================
        var endEffectorWristConfigurator = End_Effector_Wrist_Motor.getConfigurator();
        var endEffectorLimitConfigs = new CurrentLimitsConfigs();

        endEffectorLimitConfigs.StatorCurrentLimit = 120;
        endEffectorLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorWristConfigurator.apply(endEffectorLimitConfigs);

        //====================End Effector Top Roller Current Limit====================
        var endEffectorTopConfigurator = End_Effector_Top_Motor.getConfigurator();
        var endEffectorTopLimitConfigs = new CurrentLimitsConfigs();

        endEffectorTopLimitConfigs.StatorCurrentLimit = 120;
        endEffectorTopLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorTopConfigurator.apply(endEffectorTopLimitConfigs);

        //====================End Effector Bottom Indexer Current Limit====================
        var endEffectorBottomConfigurator = End_Effector_Bottom_Motor.getConfigurator();
        var endEffectorBottomLimitConfigs = new CurrentLimitsConfigs();

        endEffectorBottomLimitConfigs.StatorCurrentLimit = 120;
        endEffectorBottomLimitConfigs.StatorCurrentLimitEnable = true;
        endEffectorBottomConfigurator.apply(endEffectorBottomLimitConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("End Effector Wrist Encoder", getEndEffectorWristEncoder());
    }
    
    //====================End Effector Wrist Methods====================
    public double getEndEffectorWristEncoder() {
        return End_Effector_Wrist_Motor.getPosition().getValueAsDouble();
    }

    public void setEndEffectorWristMotorSpeed(double speed) {
        End_Effector_Wrist_Motor.set(speed);
    }

    //====================End Effector Roller Methods====================
    public void setEndEffectorRollerMotorSpeed(double speed) {
        End_Effector_Top_Motor.set(speed);
        End_Effector_Bottom_Motor.set(speed);
    }

    //====================End Effector Claw Methods====================
    public void openClaws() {
        Claw_Solenoid_1.set(DoubleSolenoid.Value.kForward);
        Claw_Solenoid_2.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaws() {
        Claw_Solenoid_1.set(DoubleSolenoid.Value.kReverse);
        Claw_Solenoid_2.set(DoubleSolenoid.Value.kReverse);
    }
}