package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class ElevatorPlace extends Command {
    private final Elevator elevator;
    private double setpoint;
    private double speedMultiplier;
    private double turnMultiplier;
    private final Drivetrain drivetrain;
    private final XboxController controller;

    public ElevatorPlace(Elevator elevator, double setpoint, Drivetrain drivetrain, XboxController controller, double speedMultiplier, double turnMultiplier) {
        this.elevator = Elevator.getInstance();
        this.setpoint = setpoint;
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.speedMultiplier = speedMultiplier;
        this.turnMultiplier = turnMultiplier;
        addRequirements(elevator);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(setpoint);
        System.out.println("ElevatorPIDCmd Started");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
        System.out.println("ElevatorPIDCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorPIDCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}