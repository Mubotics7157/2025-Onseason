package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PhysConstants;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainRightAlign extends Command {
    private final Drivetrain drivetrain;
    
    public DrivetrainRightAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("DrivetrainRightAlign Online");
    }

    @Override
    public void execute() {
        double xSpeed = drivetrain.limelight_vertical_proportional();
        double ySpeed = drivetrain.right_pole_limelight_horizontal_proportional();

        SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(PhysConstants.Absolute_Zero));
        System.out.println("DrivetrainRightAlign Executing");
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DrivetrainRightAlign Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}