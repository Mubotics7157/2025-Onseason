package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.PhysConstants;

public class EndEffectorWrist extends Command {
    private final EndEffector endEffector;
    private double setpoint;

    public EndEffectorWrist(EndEffector endEffector, double setpoint) {
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;

        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorWristSetpoint(setpoint);
        System.out.println("EndEffectorWrist Online");
    }

    @Override
    public void execute() {
        endEffector.goToEndEffectorWristSetpoint();
        System.out.println("EndEffectorWrist Executing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EndEffectorWrist Offline");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(endEffector.getEndEffectorWristEncoder() - setpoint) < PhysConstants.PID_Setpoint_Tolerance; 
    }
}