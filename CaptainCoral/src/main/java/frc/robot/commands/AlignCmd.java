package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class AlignCmd extends Command {
    private final Drivetrain drivetrain;
    private final double xSpeed;
    private final double ySpeed;
    private final double rotSpeed;
    
    public AlignCmd(Drivetrain drivetrain) {
        this.drivetrain = TunerConstants.createDrivetrain();    
        this.xSpeed = drivetrain.limelight_range_proportional();
        this.ySpeed = drivetrain.limelight_range_proportional();
        this.rotSpeed = drivetrain.limelight_aim_proportional();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCmd Started");
    }

    @Override
    public void execute() {
        SwerveRequest.FieldCentric drivetrainRequest = new SwerveRequest.FieldCentric()
        .withDeadband(1 * 0.1).withRotationalDeadband(1 * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(drivetrain.limelight_aim_proportional())
        .withVelocityY(drivetrain.limelight_aim_proportional())
        .withRotationalRate(drivetrain.limelight_range_proportional()));
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