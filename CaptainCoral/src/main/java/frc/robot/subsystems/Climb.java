package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.DeviceConstants;

public class Climb extends SubsystemBase {
    private final TalonFX Climb_Motor = new TalonFX(DeviceConstants.CLIMB_MOTOR_DEVICE_ID);

    public static Climb getInstance() {
        return instance;
    }

    private static Climb instance = new Climb();

    public Climb() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder", getClimbEncoder());
    }

    //====================Climb Methods====================
    public double getClimbEncoder() {
        return Climb_Motor.getPosition().getValueAsDouble();
    }

    public void setClimbMotorSpeed(double speed) {
        Climb_Motor.set(speed);
    }
}