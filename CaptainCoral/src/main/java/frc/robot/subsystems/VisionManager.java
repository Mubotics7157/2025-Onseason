package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class VisionManager extends SubsystemBase {
  private Drivetrain drivetrain = RobotContainer.drivetrain;

  public static VisionManager getInstance() {
    return instance;
  }

  private static VisionManager instance = new VisionManager();

  public VisionManager() {
    System.out.println("====================Vision Manager Online====================");
  }

  @Override
  public void periodic() {
    // updateOdometry();
  }

  public void updateOdometry() {

    LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    boolean doRejectUpdate = false;

    if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 360) {
      doRejectUpdate = true;
    }

    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      drivetrain.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds),
          VecBuilder.fill(0.7, 0.7, 999999999)); // Decrease if laggy
    }
  }
}