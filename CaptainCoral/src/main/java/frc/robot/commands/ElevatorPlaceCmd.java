package frc.robot.commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.KinematicsConstants;

public class ElevatorPlaceCmd extends Command {
    private final Elevator elevator;
    private double setpoint;
    private double speedMultiplier;
    private double turnMultiplier;
    private final Drivetrain drivetrain;
    private final XboxController controller;

    public ElevatorPlaceCmd(Elevator elevator, double setpoint, Drivetrain drivetrain, XboxController controller, double speedMultiplier, double turnMultiplier) {
        this.elevator = Elevator.getInstance();
        this.setpoint = setpoint;
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.speedMultiplier = speedMultiplier;
        this.turnMultiplier = turnMultiplier;
        addRequirements(elevator);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(setpoint);

        // if(setpoint == KinematicsConstants.Elevator_L1_Setpoint){
        //     EndEffector.getInstance().editRollerSpeed(.1 * KinematicsConstants.End_Effector_Score_Speed);
        // }
        // else{
        //     EndEffector.getInstance().editRollerSpeed(KinematicsConstants.End_Effector_Score_Speed);

        // }
        System.out.println("ElevatorPIDCmd Started");
    }

    @Override
    public void execute() {
        elevator.goToElevatorSetpoint();
        drivetrain.slowDrivetrain(controller, speedMultiplier, turnMultiplier);
        System.out.println("ElevatorPIDCmd Ongoing");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorPIDCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}