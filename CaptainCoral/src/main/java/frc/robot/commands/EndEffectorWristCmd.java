package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.KinematicsConstants;
import frc.robot.subsystems.EndEffector;

public class EndEffectorWristCmd extends Command {
    private final EndEffector endEffector;
    private final ProfiledPIDController profiledPIDController;
    private final ElevatorFeedforward feedforward;
    private double setpoint;

    public EndEffectorWristCmd(EndEffector endEffector, double setpoint) {
        this.endEffector = EndEffector.getInstance();
        this.setpoint = setpoint;
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.25, 0.7);
        profiledPIDController = new ProfiledPIDController(0.0001, 0.0, 0.0, constraints);
        feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);

        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        profiledPIDController.setGoal(setpoint);
        System.out.println("ElevatorPIDCmd Started");
    }

    @Override
    public void execute() {
        double currentPosition = endEffector.getEndEffectorWristEncoder();
            double output = profiledPIDController.calculate(currentPosition);
            double motorOutput = output + feedforward.calculate(profiledPIDController.getSetpoint().velocity);
            endEffector.setEndEffectorWristMotorSpeed(motorOutput);    
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorWristMotorSpeed(KinematicsConstants.absoluteZero);
    }

    public void updateSetpoint(double newSetpoint) {
         this.setpoint = newSetpoint;
         profiledPIDController.setGoal(newSetpoint);
    }
}