package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCmd extends Command {
    private final Elevator elevator;
    private final double elevatorSetpoint;

    public ElevatorPIDCmd(Elevator elevator, double elevatorSetpoint) {
        this.elevator = Elevator.getInstance();
        this.elevatorSetpoint = elevatorSetpoint;
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("ElevatorPIDCmd Online");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorPIDCmd Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}