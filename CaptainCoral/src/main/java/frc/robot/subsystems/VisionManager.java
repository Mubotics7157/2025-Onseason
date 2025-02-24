package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionManager extends SubsystemBase {
    
    public static VisionManager getInstance() {
        return instance;
    }

    private static VisionManager instance = new VisionManager();

    public VisionManager() {}

    @Override
    public void periodic() {
        Pose2d pose = getPose();

        if (pose == null) {
            return;
        }

        SmartDashboard.putNumber("x Pose", pose.getX());
        SmartDashboard.putNumber("y Pose", pose.getY());
        SmartDashboard.putNumber("rot Pose", pose.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    
        if (results.targets_Fiducials.length > 0) {
            LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
            return tag.getCameraPose_TargetSpace().toPose2d();
        } else {
            return null;
        }
    }
}