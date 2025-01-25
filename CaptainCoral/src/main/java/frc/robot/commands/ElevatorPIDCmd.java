package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorPIDCmd extends Command {
    private final Elevator elevator;
    private double setpoint;

    public ElevatorPIDCmd(Elevator elevator, double setpoint) {
        this.elevator = Elevator.getInstance();
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(setpoint);
        SmartDashboard.putString("Elevator State","ElevatorPIDCmd Started");
    }
}