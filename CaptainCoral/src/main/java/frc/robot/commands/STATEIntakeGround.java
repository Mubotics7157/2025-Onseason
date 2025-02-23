package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.KinematicsConstants;

public class STATEIntakeGround extends Command {
    private final double speed;
    private final EndEffector endEffector;
    private final double setpoint;

    private final Intake intake;
    private final double intakeSetpoint;
    private final double intakeSpeed;

    public STATEIntakeGround(EndEffector endEffector, double speed, double setpoint, Intake intake, double intakeSetpoint, double intakeSpeed) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;

        this.intake = Intake.getInstance();
        this.intakeSetpoint = intakeSetpoint;
        this.intakeSpeed = intakeSpeed;

        addRequirements(endEffector);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeWristSetpoint(intakeSetpoint);
        endEffector.setEndEffectorWristSetpoint(setpoint);
        System.out.println("GroundIntakeCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;

        intake.setIntakeRollerMotorSpeed(intakeSpeed);
        intake.setIndexerMotorSpeed(intakeSpeed);

        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

        if (endEffector.getEndEffectorSensorReading() == true) {
            endEffector.setEndEffectorRollerMotorSpeed(KinematicsConstants.Absolute_Zero);
        } else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        }

        endEffector.goToEndEffectorWristSetpoint();

        intake.goToIntakeWristSetpoint();
            
        System.out.println("GroundIntakeCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(KinematicsConstants.Absolute_Zero);
        System.out.println("GroundIntakeCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}