package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.DeviceConstants;

public class Climb extends SubsystemBase {
    private final TalonFX Climb_Master_Motor = new TalonFX(DeviceConstants.CLIMB_MASTER_MOTOR_DEVICE_ID);
    private final TalonFX Climb_Slave_Motor = new TalonFX(DeviceConstants.CLIMB_SLAVE_MOTOR_DEVICE_ID);

    public static Climb getInstance() {
        return instance;
    }

    private static Climb instance = new Climb();

    public Climb() {
        Climb_Master_Motor.setNeutralMode(NeutralModeValue.Brake);
        Climb_Slave_Motor.setNeutralMode(NeutralModeValue.Brake);

        //====================Climb Current Limit====================
        var climbMasterConfigurator = Climb_Master_Motor.getConfigurator();
        var climbMasterLimitConfigs = new CurrentLimitsConfigs();

        climbMasterLimitConfigs.StatorCurrentLimit = 120;
        climbMasterLimitConfigs.StatorCurrentLimitEnable = true;
        climbMasterConfigurator.apply(climbMasterLimitConfigs);

          //====================Climb Current Limit====================
          var climbSlaveConfigurator = Climb_Slave_Motor.getConfigurator();
          var climbSlaveLimitConfigs = new CurrentLimitsConfigs();
  
          climbSlaveLimitConfigs.StatorCurrentLimit = 120;
          climbSlaveLimitConfigs.StatorCurrentLimitEnable = true;
          climbSlaveConfigurator.apply(climbSlaveLimitConfigs);
    }

    @Override
    public void periodic() {}

    //====================Climb Methods====================
    public void setClimbMotorSpeed(double speed) {
        Climb_Master_Motor.set(speed);
        Climb_Slave_Motor.set(speed);
    }
}