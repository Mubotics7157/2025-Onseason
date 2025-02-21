package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.KinematicsConstants;

public class GroundIntakeCmd extends Command {
    private final double speed;
    private final EndEffector endEffector;
    private final double setpoint;

    public GroundIntakeCmd(EndEffector endEffector, double speed, double setpoint) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setEndEffectorSetpoint(setpoint);
        System.out.println("GroundIntakeCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

        if (endEffector.getEndEffectorSensorReading() == true) {
            endEffector.setEndEffectorRollerMotorSpeed(KinematicsConstants.Absolute_Zero);
        } else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        }

        endEffector.goToEndEffectorSetpoint();
            
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