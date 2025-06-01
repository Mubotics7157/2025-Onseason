package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class EndEffectorPIDCmd extends Command {
    private final EndEffector endEffector;
    private final double endEffectorSetpoint;

    public EndEffectorPIDCmd(EndEffector endEffector, double endEffectorSetpoint) {
        this.endEffector = EndEffector.getInstance();
        this.endEffectorSetpoint = endEffectorSetpoint;

        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorWristSetpoint(endEffectorSetpoint);
        System.out.println("EndEffectorPIDCmd Online");
    }

    @Override
    public void execute() {
        endEffector.goToEndEffectorWristSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EndEffectorPIDCmd Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}