package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.PhysConstants;

public class IntakeRunCmd extends Command {
    private final double speed;
    private final Intake intake;

    public IntakeRunCmd(Intake intake, double speed) {
        this.speed = speed;
        this.intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeRun Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        intake.setIntakeRollerMotorSpeed(motorSpeed);
        intake.setIndexerMotorSpeed(motorSpeed);

        System.out.println("IntakeRun Executing");
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(PhysConstants.Absolute_Zero);
        intake.setIndexerMotorSpeed(PhysConstants.Absolute_Zero);
        System.out.println("IntakeRun Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}