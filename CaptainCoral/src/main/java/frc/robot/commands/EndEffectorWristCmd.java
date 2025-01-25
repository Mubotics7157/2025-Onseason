package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorWristCmd extends Command {
    private final EndEffector endEffector;
    private double setpoint;

    public EndEffectorWristCmd(EndEffector endEffector, double setpoint) {
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setSetpoint(setpoint);
        SmartDashboard.putString("Elevator State","ElevatorPIDCmd Started");
    }
}