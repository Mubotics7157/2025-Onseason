package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.KinematicsConstants;

public class EndEffectorScoreCmd extends Command {
    private final double speed;
    private final EndEffector endEffector;

    public EndEffectorScoreCmd(EndEffector endEffector, double speed) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        System.out.println("EndEffectorScoreCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
            
        System.out.println("EndEffectorScoreCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(KinematicsConstants.Absolute_Zero);
        System.out.println("EndEffectorScoreCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}