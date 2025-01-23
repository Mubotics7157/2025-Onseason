package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.DeviceConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX Elevator_Master_Motor = new TalonFX(DeviceConstants.ELEVATOR_MASTER_MOTOR_DEVICE_ID);
    private final TalonFX Elevator_Slave_Motor = new TalonFX(DeviceConstants.ELEVATOR_SLAVE_MOTOR_DEVICE_ID);

    public static Elevator getInstance() {
        return instance;
    }

    private static Elevator instance = new Elevator();

    public Elevator() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
    }

    //====================Elevator Methods====================
    public double getElevatorEncoder() {
        return Elevator_Master_Motor.getPosition().getValueAsDouble();
    }

    public void setElevatorMotorSpeed(double speed) {
        Elevator_Master_Motor.set(speed);
        Elevator_Slave_Motor.set(speed);
    }
}