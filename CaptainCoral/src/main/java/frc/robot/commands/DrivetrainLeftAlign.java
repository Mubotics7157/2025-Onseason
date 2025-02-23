package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.KinematicsConstants;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainLeftAlign extends Command {
    private final Drivetrain drivetrain;
    
    public DrivetrainLeftAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("DrivetrainLeftAlign Online");
    }

    @Override
    public void execute() {
        double xSpeed = drivetrain.limelight_vertical_proportional();
        double ySpeed = drivetrain.left_pole_limelight_horizontal_proportional();

       SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(KinematicsConstants.Absolute_Zero));
        System.out.println("DrivetrainLeftAlign Executing");
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DrivetrainLeftAlign Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}