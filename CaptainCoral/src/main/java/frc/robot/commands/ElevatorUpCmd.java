package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.KinematicsConstants;

public class ElevatorUpCmd extends Command {
    private final Elevator elevator;
    private double setpoint;
    private final Drivetrain drivetrain;

    public ElevatorUpCmd(Elevator elevator, double setpoint, Drivetrain drivetrain) {
        this.elevator = Elevator.getInstance();
        this.setpoint = setpoint;
        this.drivetrain = drivetrain;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(setpoint);
        System.out.println("ElevatorPIDCmd Started");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();

        SwerveRequest.FieldCentric drivetrainRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        drivetrain.setControl(drivetrainRequest.withVelocityX(1.0).withVelocityY(1.0).withRotationalRate(1.0));

        System.out.println("ElevatorPIDCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorPIDCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return elevator.getElevatorMasterEncoder() - setpoint < KinematicsConstants.PID_Setpoint_Tolerance;
    }
}