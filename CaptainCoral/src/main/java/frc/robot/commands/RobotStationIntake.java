package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotStationIntake extends Command {
    private final EndEffector endEffector;
    private final double endEffectorSetpoint;
    private final double endEffectorSpeed;

    private final Elevator elevator;
    private final double elevatorSetpoint;

    public RobotStationIntake(EndEffector endEffector, double endEffectorSetpoint, double endEffectorSpeed, Elevator elevator, double elevatorSetpoint) {
        this.endEffector = EndEffector.getInstance();
        this.endEffectorSetpoint = endEffectorSetpoint;
        this.endEffectorSpeed = endEffectorSpeed;

        this.elevator = Elevator.getInstance();
        this.elevatorSetpoint = elevatorSetpoint;
        
        addRequirements(elevator);
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorWristSetpoint(endEffectorSetpoint);
        elevator.setElevatorSetpoint(elevatorSetpoint);
        System.out.println("RobotStationIntake Online");
    }

    @Override
    public void execute() {
        double motorSpeed = endEffectorSpeed;

        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

        if (endEffector.getEndEffectorFrontPhotoElectricReading() == true) {
            endEffector.setEndEffectorRollerMotorSpeed(Constants.Absolute_Zero);
        } else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        }

        endEffector.goToEndEffectorWristSetpoint();
        elevator.goToElevatorSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("RobotStationIntake Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}