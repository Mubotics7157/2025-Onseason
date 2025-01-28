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
    
    public AlignCmd(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCmd Started");
    }

    @Override
    public void execute() {
        double xySpeed = drivetrain.limelight_range_proportional();
        //double rotSpeed = drivetrain.limelight_aim_proportional();


        SwerveRequest.FieldCentric drivetrainRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(xySpeed)
        .withVelocityY(0.0)
        .withRotationalRate(0.0));
        System.out.println(xySpeed);
        //System.out.println(rotSpeed);
        System.out.println("Ongoing Align");
        }

    @Override
    public void end(boolean interrupted) {
        // SwerveRequest.FieldCentric drivetrainRequest = new SwerveRequest.FieldCentric()
        // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        // .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        // drivetrain.setControl(drivetrainRequest
        // .withVelocityX(0.0)
        // .withVelocityY(0.0)
        // .withRotationalRate(0.0));
        System.out.println("AlignCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}