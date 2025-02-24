package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.DeviceConstants;
import frc.robot.PhysConstants;

public class EndEffectorRun extends Command {
    private final double speed;
    private final EndEffector endEffector;
    private final XboxController controller;

    public EndEffectorRun(EndEffector endEffector, double speed, XboxController controller) {
        this.speed = speed;
        this.endEffector = EndEffector.getInstance();
        this.controller = controller;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        System.out.println("EndEffectorRun Online");
    }

    @Override
    public void execute() {
        double motorSpeed = speed;
        endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);

        //End Effector Sensor (Roller Stop)
        if (endEffector.getEndEffectorSensorReading() == true) {
            endEffector.setEndEffectorRollerMotorSpeed(PhysConstants.Absolute_Zero);
        } else {
            endEffector.setEndEffectorRollerMotorSpeed(motorSpeed);
        }

        //End Effector Sensor (Controller Rumble)
        if (endEffector.getEndEffectorSensorReading() == true) {
            controller.setRumble(XboxController.RumbleType.kLeftRumble, DeviceConstants.DRIVER_CONTROLLER_RUMBLE);
            controller.setRumble(XboxController.RumbleType.kRightRumble, DeviceConstants.DRIVER_CONTROLLER_RUMBLE);
        } else {
            controller.setRumble(XboxController.RumbleType.kLeftRumble, PhysConstants.Absolute_Zero);
            controller.setRumble(XboxController.RumbleType.kRightRumble, PhysConstants.Absolute_Zero);
        }
            
        System.out.println("EndEffectorRun Executing");
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setEndEffectorRollerMotorSpeed(PhysConstants.Absolute_Zero);
        controller.setRumble(XboxController.RumbleType.kLeftRumble, PhysConstants.Absolute_Zero);
        controller.setRumble(XboxController.RumbleType.kRightRumble, PhysConstants.Absolute_Zero);
        System.out.println("EndEffectorRun Offline");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}