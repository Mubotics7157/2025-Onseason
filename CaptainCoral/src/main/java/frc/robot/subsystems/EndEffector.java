package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DeviceConstants;
import frc.robot.KinematicsConstants;

public class EndEffector extends SubsystemBase {
    private final TalonFX End_Effector_Wrist_Master_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_WRIST_MASTER_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Wrist_Slave_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_WRIST_SLAVE_MOTOR_DEVICE_ID);

    private final TalonFX End_Effector_Top_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_TOP_MOTOR_DEVICE_ID);
    private final TalonFX End_Effector_Bottom_Motor = new TalonFX(DeviceConstants.END_EFFECTOR_BOTTOM_MOTOR_DEVICE_ID);

    // private final DoubleSolenoid Claw_Solenoid_1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DeviceConstants.CLAW_SOLENOID_1_FORWARD_CHANNEL, DeviceConstants.CLAW_SOLENOID_1_REVERSE_CHANNEL);
    // private final DoubleSolenoid Claw_Solenoid_2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DeviceConstants.CLAW_SOLENOID_2_FORWARD_CHANNEL, DeviceConstants.CLAW_SOLENOID_2_REVERSE_CHANNEL);

    private double setpoint;

    public static EndEffector getInstance() {
        return instance;
    }

    private static EndEffector instance = new EndEffector();

    public EndEffector() {
        ////====================Inverted Settings====================
        var invertedConfiguration = new TalonFXConfiguration();
        invertedConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        End_Effector_Wrist_Slave_Motor.getConfigurator().apply(invertedConfiguration);

        //====================End Effector Wrist Motion Magic====================
        End_Effector_Wrist_Master_Motor.setPosition(0.0);
        End_Effector_Wrist_Slave_Motor.setPosition(0.0);

        var endEffectorWristMotorConfigs = new TalonFXConfiguration();

        var generalSlotConfigs = endEffectorWristMotorConfigs.Slot0;
        generalSlotConfigs.kS = 0.0; // Add 0.25 V output to overcome static friction
        generalSlotConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        generalSlotConfigs.kA = 0.05; // An acceleration of 1 rps/s requires 0.01 V output
        generalSlotConfigs.kP = 2.5; // A position error of 2.5 rotations results in 12 V output 0.0265
        generalSlotConfigs.kI = 0.0; // no output for integrated error
        generalSlotConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = endEffectorWristMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 32; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 64; // Target jerk of 1600 rps/s/s (0.1 seconds)

        End_Effector_Wrist_Master_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);
        End_Effector_Wrist_Slave_Motor.getConfigurator().apply(endEffectorWristMotorConfigs);

        //====================End Effector Wrist Master Current Limit====================
        // var endEffectorWristMasterConfigurator = End_Effector_Wrist_Master_Motor.getConfigurator();
        // var endEffectorMasterLimitConfigs = new CurrentLimitsConfigs();

        // endEffectorMasterLimitConfigs.StatorCurrentLimit = 120;
        // endEffectorMasterLimitConfigs.StatorCurrentLimitEnable = true;
        // endEffectorWristMasterConfigurator.apply(endEffectorMasterLimitConfigs);

        //====================End Effector Wrist Slave Current Limit====================
        // var endEffectorWristSlaveConfigurator = End_Effector_Wrist_Slave_Motor.getConfigurator();
        // var endEffectorSlaveLimitConfigs = new CurrentLimitsConfigs();

        // endEffectorSlaveLimitConfigs.StatorCurrentLimit = 120;
        // endEffectorSlaveLimitConfigs.StatorCurrentLimitEnable = true;
        // endEffectorWristSlaveConfigurator.apply(endEffectorSlaveLimitConfigs);

        // //====================End Effector Top Roller Current Limit====================
        // var endEffectorTopConfigurator = End_Effector_Top_Motor.getConfigurator();
        // var endEffectorTopLimitConfigs = new CurrentLimitsConfigs();

        // endEffectorTopLimitConfigs.StatorCurrentLimit = 120;
        // endEffectorTopLimitConfigs.StatorCurrentLimitEnable = true;
        // endEffectorTopConfigurator.apply(endEffectorTopLimitConfigs);

        // //====================End Effector Bottom Indexer Current Limit====================
        // var endEffectorBottomConfigurator = End_Effector_Bottom_Motor.getConfigurator();
        // var endEffectorBottomLimitConfigs = new CurrentLimitsConfigs();

        // endEffectorBottomLimitConfigs.StatorCurrentLimit = 120;
        // endEffectorBottomLimitConfigs.StatorCurrentLimitEnable = true;
        // endEffectorBottomConfigurator.apply(endEffectorBottomLimitConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("End Effector Wrist Master Encoder", getEndEffectorWristMasterEncoder());
        SmartDashboard.putNumber("End Effector Top Encoder", getEndEffectorWristSlaveEncoder());
    }
    
    //====================End Effector Wrist Methods====================
    public double getEndEffectorWristMasterEncoder() {
        return End_Effector_Wrist_Master_Motor.getPosition().getValueAsDouble();
    }

    public double getEndEffectorWristSlaveEncoder() {
        return End_Effector_Wrist_Slave_Motor.getPosition().getValueAsDouble();
    }

    public void setEndEffectorWristMotorSpeed(double speed) {
        System.out.println("End Effector Wrist Motor Speed: " + speed);
        End_Effector_Wrist_Master_Motor.set(speed);
        End_Effector_Wrist_Slave_Motor.set(speed);
    }

    public void setEndEffectorSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToEndEffectorSetpoint() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(KinematicsConstants.absoluteZero);
        End_Effector_Wrist_Master_Motor.setControl(m_request.withPosition(this.setpoint));
        //End_Effector_Wrist_Slave_Motor.setControl(m_request.withPosition(this.setpoint));
    }

    //====================End Effector Roller Methods====================
    public void setEndEffectorRollerMotorSpeed(double speed) {
        End_Effector_Top_Motor.set(speed);
        End_Effector_Bottom_Motor.set(speed);
    }

    //====================End Effector Claw Methods====================
    // public void openClaws() {
    //     Claw_Solenoid_1.set(DoubleSolenoid.Value.kForward);
    //     Claw_Solenoid_2.set(DoubleSolenoid.Value.kForward);
    // }

    // public void closeClaws() {
    //     Claw_Solenoid_1.set(DoubleSolenoid.Value.kReverse);
    //     Claw_Solenoid_2.set(DoubleSolenoid.Value.kReverse);
    // }
}