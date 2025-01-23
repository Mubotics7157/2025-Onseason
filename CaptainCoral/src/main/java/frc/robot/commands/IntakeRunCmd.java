package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.KinematicsConstants;

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
        System.out.println("IntakeRunCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        intake.setIntakeRollerMotorSpeed(motorSpeed);
        intake.setIndexerMotorSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(KinematicsConstants.absoluteZero);
        intake.setIndexerMotorSpeed(KinematicsConstants.absoluteZero);
        System.out.println("IntakeRunCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}