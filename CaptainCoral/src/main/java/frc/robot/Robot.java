package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionManager;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  
  private final boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
      super.robotInit();
      //Intake.getInstance().zeroIntakeWristWithAbsolute();
      VisionManager.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Match Time Log
    double matchTime = Timer.getMatchTime();
    SmartDashboard.putNumber("Match Time", matchTime);
   
    //Battery Voltage Log
    double batteryVoltage = RobotController.getBatteryVoltage();
    SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
  }

  @Override
  public void disabledInit() {
    //Intake.getInstance().zeroIntakeWristWithAbsolute();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //Intake.getInstance().zeroIntakeWristWithAbsolute();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //Intake.getInstance().zeroIntakeWristWithAbsolute();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}