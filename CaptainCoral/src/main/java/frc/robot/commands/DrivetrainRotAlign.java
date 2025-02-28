package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainRotAlign extends Command {
    private final Drivetrain drivetrain;
    
    public DrivetrainRotAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("DrivetrainRotAlign Online");
    }

    @Override
    public void execute() {
        double rotSpeed = drivetrain.rotation_limelight_proportional();

        SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(Constants.Absolute_Zero)
        .withVelocityY(Constants.Absolute_Zero)
        .withRotationalRate(rotSpeed));
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DrivetrainRotAlign Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}