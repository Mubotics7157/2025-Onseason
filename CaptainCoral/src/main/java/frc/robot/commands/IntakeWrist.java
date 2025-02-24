package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.PhysConstants;

public class IntakeWrist extends Command {
    private final Intake intake;
    private double setpoint;

    public IntakeWrist(Intake intake, double setpoint) {
        this.intake = Intake.getInstance();
        this.setpoint = setpoint;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeWristSetpoint(setpoint);
        System.out.println("IntakeWrist Online");
    }

    @Override
    public void execute() {
        intake.goToIntakeWristSetpoint();
        System.out.println("IntakeWrist Executing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IntakeWrist Offline");
    }

    @Override
    public boolean isFinished() {
        return intake.getIntakeWristEncoder() - setpoint < PhysConstants.PID_Setpoint_Tolerance;
    }
}