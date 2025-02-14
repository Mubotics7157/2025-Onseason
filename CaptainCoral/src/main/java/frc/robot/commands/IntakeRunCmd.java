package frc.robot.commands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.DeviceConstants;
import frc.robot.KinematicsConstants;

public class IntakeRunCmd extends Command {
    private final double speed;
    private final Intake intake;
    private final XboxController controller;

    public IntakeRunCmd(Intake intake, double speed, XboxController controller) {
        this.speed = speed;
        this.intake = Intake.getInstance();
        this.controller = controller;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeRunCmd Started");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        intake.setIntakeRollerMotorSpeed(motorSpeed);
        intake.setIndexerMotorSpeed(motorSpeed);
        System.out.println("IntakeRunCmd Ongoing");

        if (intake.getIndexerSensorReading() == true) {
            //Stow Intake
            controller.setRumble(XboxController.RumbleType.kLeftRumble, DeviceConstants.DRIVER_CONTROLLER_RUMBLE);
            controller.setRumble(XboxController.RumbleType.kRightRumble, DeviceConstants.DRIVER_CONTROLLER_RUMBLE);
        } else {
            intake.setIntakeRollerMotorSpeed(motorSpeed);
            intake.setIndexerMotorSpeed(motorSpeed);
            controller.setRumble(XboxController.RumbleType.kLeftRumble, KinematicsConstants.Absolute_Zero);
            controller.setRumble(XboxController.RumbleType.kRightRumble, KinematicsConstants.Absolute_Zero);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeRollerMotorSpeed(KinematicsConstants.Absolute_Zero);
        intake.setIndexerMotorSpeed(KinematicsConstants.Absolute_Zero);
        controller.setRumble(XboxController.RumbleType.kLeftRumble, KinematicsConstants.Absolute_Zero);
        controller.setRumble(XboxController.RumbleType.kRightRumble, KinematicsConstants.Absolute_Zero);
        System.out.println("IntakeRunCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}