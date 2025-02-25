package frc.robot.commands;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionManager;

public class DrivetrainXPoseAlign extends Command {
    private final Drivetrain drivetrain;
    private final VisionManager visionManager;
    
    public DrivetrainXPoseAlign(Drivetrain drivetrain, VisionManager visionManager) {
        this.drivetrain = drivetrain;
        this.visionManager = visionManager;

        addRequirements(drivetrain);
        addRequirements(visionManager);

        SmartDashboard.putNumber("DrivePose Horizontal kP", Constants.Drivetrain_X_Pose_kP);
    }

    @Override
    public void initialize() {
        System.out.println("DrivetrainHorizontalPose Online");
    }

    public double getXSpeed() { 
        Pose2d currentPose = visionManager.getPose();
        
        if (currentPose == null) {
            return 0.0;
        } else {
            return currentPose.getX() * SmartDashboard.getNumber("DrivePose Horizontal kP", Constants.Drivetrain_X_Pose_kP) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
      }

    @Override
    public void execute() {
       SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(getXSpeed())
        .withVelocityY(0.0)
        .withRotationalRate(0.0));

        System.out.println("DrivetrainHorizontalPose Executing");
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DrivetrainHorizontalPose Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}