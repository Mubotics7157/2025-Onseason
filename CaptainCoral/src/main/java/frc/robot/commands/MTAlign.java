package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MTAlign extends Command {
    private final Drivetrain drivetrain;
    private final ProfiledPIDController xPidController = new ProfiledPIDController(0.0, 0, 0, new Constraints(0.0, 0.0));

    private final ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.0, 0, 0, new Constraints(0.0, 0.0));

    public MTAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        //HotRefreshXAlignPID
        SmartDashboard.putNumber("XAlign kP", 0.0);
        SmartDashboard.putNumber("XAlign kI", 0.0);
        SmartDashboard.putNumber("XAlign kD", 0.0);
        SmartDashboard.putNumber("XAlign kVelo", 0.0);
        SmartDashboard.putNumber("XAlign kAccel", 0.0);

        //HotRefreshRotAlignPID
        // SmartDashboard.putNumber("RotAlign kP", 0.0);
        // SmartDashboard.putNumber("RotAlign kI", 0.0);
        // SmartDashboard.putNumber("RotAlign kD", 0.0);
        // SmartDashboard.putNumber("RotAlign kVelo", 0.0);
        // SmartDashboard.putNumber("RotAlign kAccel", 0.0);
    }

    @Override
    public void initialize() {
        System.out.println("MTAlign Online");   
        HotRefreshMTAlignPID();
    }

    @Override
    public void execute() {   

        double xSpeed = xPidController.calculate(drivetrain.getState().Pose.getX(), 5.35); 
                   
        //double rotSpeed = rotationPIDController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), Math.toRadians(0.0));

        SwerveRequest.FieldCentric drivetrainRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
        .withVelocityX(Constants.Absolute_Zero)
        .withVelocityY(xSpeed)
        .withRotationalRate(Constants.Absolute_Zero));
        }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MTAlign Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void HotRefreshMTAlignPID() {
        xPidController.setP(SmartDashboard.getNumber("XAlign kP", 0.0));
        xPidController.setI(SmartDashboard.getNumber("XAlign kI", 0.0));
        xPidController.setD(SmartDashboard.getNumber("XAlign kD", 0.0));
        xPidController.setConstraints(new Constraints(SmartDashboard.getNumber("XAlign kVelo", 0.0), SmartDashboard.getNumber("XAlign kAccel", 0.0)));

        // rotationPIDController.setP(SmartDashboard.getNumber("RotAlign kP", 0.0));
        // rotationPIDController.setI(SmartDashboard.getNumber("RotAlign kI", 0.0));
        // rotationPIDController.setD(SmartDashboard.getNumber("RotAlign kD", 0.0));
        // rotationPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("RotAlign kVelo", 0.0), SmartDashboard.getNumber("RotAlign kAccel", 0.0)));
        System.out.println("HotRefreshMTAlignPID Complete");
    }
}