package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.KinematicsConstants;

public class EndEffectorWristCmd extends Command {
    private final EndEffector endEffector;
    private double setpoint;
    private double testSetpoint;

    public EndEffectorWristCmd(EndEffector endEffector, double setpoint) {
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;
        this.testSetpoint =  SmartDashboard.getNumber("WristSetpoint", this.setpoint);

        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorSetpoint(setpoint);
        System.out.println("EndEffectorWristCmd Started");
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("data from get data", this.testSetpoint);
        endEffector.goToEndEffectorSetpoint();
        System.out.println("EndEffectorWristCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EndEffectorWristCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(endEffector.getEndEffectorWristMasterEncoder() - setpoint) < KinematicsConstants.PID_Setpoint_Tolerance; 
    }
}