package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeWristCmd extends Command {
    private final Intake intake;
    private double setpoint;

    public IntakeWristCmd(Intake intake, double setpoint) {
        this.intake = Intake.getInstance();
        this.setpoint = setpoint;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSetpoint(setpoint);
        System.out.println("EndEffectorWristCmd Started");
    }

    @Override
    public void execute() {
        intake.goToIntakeSetpoint();
        System.out.println("EndEffectorWristCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EndEffectorWristCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getIntakeWristEncoder() - setpoint) < 0.05; 
    }
}