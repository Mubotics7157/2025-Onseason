package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

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
        elevator.setElevatorSetpoint(setpoint);
        System.out.println("ElevatorPIDCmd Started");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();
        System.out.println("ElevatorPIDCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorPIDCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return elevator.getElevatorMasterEncoder() - setpoint < 0.05;

    //     if (Math.abs(elevator.getElevatorMasterEncoder() - setpoint) < 0.1) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }
    }
}