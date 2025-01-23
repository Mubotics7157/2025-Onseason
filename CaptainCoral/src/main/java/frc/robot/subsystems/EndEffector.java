package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public EndEffector() {}

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