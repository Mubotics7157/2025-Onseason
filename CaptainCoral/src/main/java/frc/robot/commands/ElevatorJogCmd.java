package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;
import frc.robot.KinematicsConstants;

public class ElevatorJogCmd extends Command {
    private final Elevator elevator;
    private final DoubleSupplier speed; 

    public ElevatorJogCmd(Elevator elevator, DoubleSupplier speed) {
        this.speed = speed;
        this.elevator = Elevator.getInstance(); 
        addRequirements(elevator); 
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorJogCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed.getAsDouble();
        elevator.setElevatorMotorSpeed(motorSpeed);
        System.out.println("ElevatorJogCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorMotorSpeed(KinematicsConstants.Absolute_Zero);
        System.out.println("ElevatorJogCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}