package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Devices;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final TalonFX Climb_Motor = new TalonFX(Devices.CLIMB_MOTOR);

    public static Climb getInstance() {
        return instance;
    }

    private static Climb instance = new Climb();

    public Climb() {
        System.out.println("====================Climb Subsystem Online====================");

        //====================Climb Subsystem====================
        var climbMotorConfigs = new TalonFXConfiguration();

        climbMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Current Limits
        var climbLimitConfigs = climbMotorConfigs.CurrentLimits;
        climbLimitConfigs.StatorCurrentLimit = Constants.Climb_Current_Limit;
        climbLimitConfigs.StatorCurrentLimitEnable = true;    

        //Applies Configs
        Climb_Motor.getConfigurator().apply(climbMotorConfigs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Motor Encoder", getClimbEncoder());
        SmartDashboard.putNumber("Climb Stator Current", getClimbStatorCurrent());   
    }

    //====================Climb Methods====================
    public void setClimbMotorSpeed(double speed) {
        Climb_Motor.set(speed);
    }

    public double getClimbEncoder() {
        return Climb_Motor.getPosition().getValueAsDouble();
    }

    public double getClimbStatorCurrent() {
        return Climb_Motor.getStatorCurrent().getValueAsDouble();
    }
}