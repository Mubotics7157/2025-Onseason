package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        intake.setSetpoint(setpoint);
        SmartDashboard.putString("Elevator State","ElevatorPIDCmd Started");
    }
}