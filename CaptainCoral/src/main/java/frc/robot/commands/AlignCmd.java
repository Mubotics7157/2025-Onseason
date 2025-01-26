package frc.robot.commands;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;

//Limelight Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AlignCmd extends Command {
    private final Drivetrain drivetrain;
    private final double xSpeed;
    private final double ySpeed;
    private final double rotSpeed;
    
    public AlignCmd(Drivetrain drivetrain) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCmd Started");
    }

    @Override
    public void execute() {
        private void goToTag() {
            double xSpeed = drivetrain.limelight_range_proportional();
            double ySpeed = drivetrain.limelight_range_proportional();
            double rotSpeed = drivetrain.limelight_aim_proportional();
        
            //Make sure this is not field relative!
            drivetrain.applyRequest(() -> drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
        System.out.println("AlignCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}