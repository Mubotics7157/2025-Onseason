package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class ElevatorPlace extends Command {
    private final Elevator elevator;
    private final double setpoint;
    private final double speedMultiplier;
    private final double turnMultiplier;
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
        System.out.println("ElevatorPlace Online");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorPlace Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}