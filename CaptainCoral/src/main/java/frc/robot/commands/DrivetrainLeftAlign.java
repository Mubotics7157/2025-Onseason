package frc.robot.commands;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionManager;

public class DrivetrainLeftAlign extends Command {
    private final Drivetrain drivetrain;
    private final VisionManager visionManager;

    ProfiledPIDController FBPIDController = new ProfiledPIDController(2.75, 0, 0.0, new Constraints(4.0, 4.0)); //2.75, 0, 0, 4
    ProfiledPIDController LRPIDController = new ProfiledPIDController(6.0, 0, 0.1, new Constraints(0.2, 0.2));
    ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0, 1.0));      
    
    boolean FBPositionHasReset = false;

    public DrivetrainLeftAlign(Drivetrain drivetrain, VisionManager visionManager) {
        this.drivetrain = drivetrain;
        this.visionManager = VisionManager.getInstance();
    
        addRequirements(drivetrain);
        addRequirements(visionManager);

        FBPIDController.setTolerance(0.02);
        LRPIDController.setTolerance(0.02);
        rotationPIDController.setTolerance(0.02);

        //HotRefreshFBAlignPID
        // SmartDashboard.putNumber("FBAlign kP", 0.0);
        // SmartDashboard.putNumber("FBAlign kI", 0.0);
        // SmartDashboard.putNumber("FBAlign kD", 0.0);
        // SmartDashboard.putNumber("FBAlign kVelo", 0.0);
        // SmartDashboard.putNumber("FBAlign kAccel", 0.0);

        //HotRefreshLRAlignPID
        // SmartDashboard.putNumber("LRAlign kP", 0.0);
        // SmartDashboard.putNumber("LRAlign kI", 0.0);
        // SmartDashboard.putNumber("LRAlign kD", 0.0);
        // SmartDashboard.putNumber("LRAlign kVelo", 0.0);
        // SmartDashboard.putNumber("LRAlign kAccel", 0.0);

        //HotRefreshRotAlignPID
        // SmartDashboard.putNumber("RotAlign kP", 0.0);
        // SmartDashboard.putNumber("RotAlign kI", 0.0);
        // SmartDashboard.putNumber("RotAlign kD", 0.0);
        // SmartDashboard.putNumber("RotAlign kVelo", 0.0);
        // SmartDashboard.putNumber("RotAlign kAccel", 0.0);
    }    

    @Override
    public void initialize() {
        System.out.println("DrivetrainPoseAlign Online");

        //HotRefreshFBAlignPID();
        //HotRefreshLRAlignPID();
        //HotRefreshRotAlignPID();
    }

    @Override
    public void execute() {
        double FB_Reading = visionManager.deriveFBPose();
        double LR_Reading = visionManager.deriveLRPose();
        double Rot_Reading = visionManager.deriveRotPose();

        if (!FBPositionHasReset && FB_Reading!= 0) {
            FBPIDController.reset(FB_Reading);
            FBPositionHasReset = true;
        } 

        //FB Speed Calculation
        double FBSpeed;
        if (FB_Reading != 0.0) {
            FBSpeed = FBPIDController.calculate(FB_Reading, -0.55); //-0.55
        } else {
            FBSpeed = 0.0;
        }

        //LR Speed Calculation
        double LRSpeed;
        if (LR_Reading != 0.0) {
            LRSpeed = -LRPIDController.calculate(LR_Reading, -0.16);
        } else {
            LRSpeed = 0.0;
        }

        // Rot Speed Calculation
        double RotSpeed;
        if (Rot_Reading != 0.0) {
            RotSpeed = rotationPIDController.calculate(Rot_Reading, 1.0);
        } else {
            RotSpeed = 0.0;
        }

        // Negate to ensure the correct direction
        RotSpeed = -RotSpeed; 

        SwerveRequest.RobotCentric drivetrainRequest = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        drivetrain.setControl(drivetrainRequest
                .withVelocityX(FBSpeed) //FBSpeed
                .withVelocityY(LRSpeed) //LRSpeed
                .withRotationalRate(RotSpeed)); //RotSpeed

        SmartDashboard.putNumber("FBSpeed", FBSpeed);
        SmartDashboard.putNumber("LRSpeed", LRSpeed);
        SmartDashboard.putNumber("RotSpeed", RotSpeed);

        SmartDashboard.putNumber("FB PID Error - Position", FBPIDController.getPositionError());
        SmartDashboard.putNumber("FB PID Error - Velocity", FBPIDController.getVelocityError());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DrivetrainPoseAlign Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // public void HotRefreshFBAlignPID() {
    //     FBPIDController.setP(SmartDashboard.getNumber("FBAlign kP", 0.0));
    //     FBPIDController.setI(SmartDashboard.getNumber("FBAlign kI", 0.0));
    //     FBPIDController.setD(SmartDashboard.getNumber("FBAlign kD", 0.0));
    //     FBPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("FBAlign kVelo", 0.0), SmartDashboard.getNumber("FBAlign kAccel", 0.0)));
    //     System.out.println("HotRefreshFBAlignPID Complete");
    // }

    // public void HotRefreshLRAlignPID() {
    //     LRPIDController.setP(SmartDashboard.getNumber("LRAlign kP", 0.0));
    //     LRPIDController.setI(SmartDashboard.getNumber("LRAlign kI", 0.0));
    //     LRPIDController.setD(SmartDashboard.getNumber("LRAlign kD", 0.0));
    //     LRPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("LRAlign kVelo", 0.0), SmartDashboard.getNumber("LRAlign kAccel", 0.0)));
    //     System.out.println("HotRefreshLRAlignPID Complete");
    // }

    // public void HotRefreshRotAlignPID() {
    //     rotationPIDController.setP(SmartDashboard.getNumber("RotAlign kP", 0.0));
    //     rotationPIDController.setI(SmartDashboard.getNumber("RotAlign kI", 0.0));
    //     rotationPIDController.setD(SmartDashboard.getNumber("RotAlign kD", 0.0));
    //     rotationPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("RotAlign kVelo", 0.0), SmartDashboard.getNumber("RotAlign kAccel", 0.0)));
    //     System.out.println("HotRefreshRotAlignPID Complete");
    // }
}