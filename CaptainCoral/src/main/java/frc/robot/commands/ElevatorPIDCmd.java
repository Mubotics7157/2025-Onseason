package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.KinematicsConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCmd extends Command {
    private final Elevator elevator;
    private final ProfiledPIDController profiledPIDController;
    private final ElevatorFeedforward feedforward;
    private double setpoint;

    public ElevatorPIDCmd(Elevator elevator, double setpoint) {
        this.elevator = Elevator.getInstance();
        this.setpoint = setpoint;
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.25, 0.7);
        profiledPIDController = new ProfiledPIDController(0.2, 0.0, 0.0, constraints);
        feedforward = new ElevatorFeedforward(0.0, 0.1, 0.006);

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        profiledPIDController.setGoal(setpoint); // Set the initial goal based on the encoder setpoint
        System.out.println("ElevatorPIDCmd Started");
    }

    @Override
    public void execute() {
        double currentPosition = elevator.getElevatorEncoder(); // Get encoder distance
            double output = profiledPIDController.calculate(currentPosition);
            double motorOutput = output + feedforward.calculate(profiledPIDController.getSetpoint().velocity);
            elevator.setElevatorMotorSpeed(motorOutput);    
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorMotorSpeed(KinematicsConstants.absoluteZero);
    }

    public void updateSetpoint(double newSetpoint) {
         this.setpoint = newSetpoint;
         profiledPIDController.setGoal(newSetpoint);
    }
}