package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Intake() {}
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