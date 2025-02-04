package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class AlignCmd extends Command {
    private final Drivetrain drivetrain;
    private double setpoint;
    
    public AlignCmd(Drivetrain drivetrain, double setpoint) {
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCmd Started");
    }

    @Override
    public void execute() {
        double xSpeed = drivetrain.limelight_range_proportional();
        double ySpeed = drivetrain.limelight_aim_proportional(setpoint);

        SwerveRequest.FieldCentric drivetrainRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(0.0));
        System.out.println("AlignCmd Ongoing");
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}