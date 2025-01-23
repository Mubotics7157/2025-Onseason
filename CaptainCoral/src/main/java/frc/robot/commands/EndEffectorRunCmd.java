package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.KinematicsConstants;

public class EndEffectorRunCmd extends Command {
    private final double speed;
    private final EndEffector endEffector;

    public EndEffectorRunCmd(EndEffector endEffector, double speed) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        System.out.println("EndEffectorRunCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(KinematicsConstants.absoluteZero);
        System.out.println("EndEffectorRunCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}