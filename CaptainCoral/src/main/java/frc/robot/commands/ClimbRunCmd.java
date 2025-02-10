package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.KinematicsConstants;

public class ClimbRunCmd extends Command {
    private final double speed;
    private final Climb climb;

    public ClimbRunCmd(Climb climb, double speed) {
        this.speed = speed;
        this.climb = Climb.getInstance();
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        System.out.println("ClimbRunCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        climb.setClimbMotorSpeed(motorSpeed);
        System.out.println("ClimbRunCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbMotorSpeed(KinematicsConstants.Absolute_Zero);
        System.out.println("ClimbRunCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}