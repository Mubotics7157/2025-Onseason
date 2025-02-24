package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.PhysConstants;

public class ClimbRun extends Command {
    private final double speed;
    private final Climb climb;

    public ClimbRun(Climb climb, double speed) {
        this.speed = speed;
        this.climb = Climb.getInstance();
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        System.out.println("ClimbRun Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        climb.setClimbMotorSpeed(motorSpeed);
        System.out.println("ClimbRun Executing");
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbMotorSpeed(PhysConstants.Absolute_Zero);
        System.out.println("ClimbRun Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}