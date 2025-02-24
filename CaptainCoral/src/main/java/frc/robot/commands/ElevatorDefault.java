package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.PhysConstants;

public class ElevatorDefault extends Command {
    private final Elevator elevator;
    private double setpoint;

    public ElevatorDefault(Elevator elevator, double setpoint) {
        this.elevator = Elevator.getInstance();
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(setpoint);
        System.out.println("ElevatorDefault Online");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();

        System.out.println("ElevatorDefault Executing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorDefault Offline");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorMasterEncoder() - setpoint) < PhysConstants.PID_Setpoint_Tolerance;
    }
}