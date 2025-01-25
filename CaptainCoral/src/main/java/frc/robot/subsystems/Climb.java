package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.DeviceConstants;

public class Climb extends SubsystemBase {
    private final TalonFX Climb_Motor = new TalonFX(DeviceConstants.CLIMB_MOTOR_DEVICE_ID);

    public static Climb getInstance() {
        return instance;
    }

    private static Climb instance = new Climb();

    public Climb() {
        //====================Climb Current Limit====================
        var climbConfigurator = Climb_Motor.getConfigurator();
        var climbLimitConfigs = new CurrentLimitsConfigs();

        climbLimitConfigs.StatorCurrentLimit = 120;
        climbLimitConfigs.StatorCurrentLimitEnable = true;
        climbConfigurator.apply(climbLimitConfigs);
    }

    @Override
    public void periodic() {}

    //====================Climb Methods====================
    public void setClimbMotorSpeed(double speed) {
        Climb_Motor.set(speed);
    }
}