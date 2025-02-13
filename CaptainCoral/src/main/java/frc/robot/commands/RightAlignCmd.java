package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.KinematicsConstants;
import frc.robot.LimelightHelpers;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class RightAlignCmd extends Command {
    private final Drivetrain drivetrain;
    
    public RightAlignCmd(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCmd Started");
    }

    @Override
    public void execute() {
        double xSpeed = drivetrain.limelight_vertical_proportional();
        double ySpeed = drivetrain.right_pole_limelight_horizontal_proportional();
        double rotSpeed = drivetrain.rotation_limelight_proportional();

        SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
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