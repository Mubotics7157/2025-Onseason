package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.KinematicsConstants;

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
        this.setpoint = SmartDashboard.getNumber("Intake Setpoint", this.setpoint);

        intake.setIntakeWristSetpoint(setpoint);
        System.out.println("IntakeWristCmd Started");
    }

    @Override
    public void execute() {
        intake.goToIntakeWristSetpoint();
        System.out.println("IntakeWristCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IntakeWristCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return intake.getIntakeWristEncoder() - setpoint < KinematicsConstants.PID_Setpoint_Tolerance;
    }
}