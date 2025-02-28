package frc.robot;

//Drivetrain Imports
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;

//Intake Imports
import frc.robot.subsystems.Intake;
import frc.robot.commands.RobotHome;
import frc.robot.commands.IntakeRunCmd;
import frc.robot.commands.IntakeWrist;
import frc.robot.commands.IntakeWristJog;
import frc.robot.commands.RobotAlgaeIntake;
import frc.robot.commands.RobotAutoPrepScore;
//End Effector Imports
import frc.robot.subsystems.EndEffector;
import frc.robot.commands.EndEffectorWrist;
import frc.robot.commands.EndEffectorWristJog;
import frc.robot.commands.RobotIntakeGround;
import frc.robot.commands.RobotPrepScore;
import frc.robot.commands.RobotStationIntake;
import frc.robot.commands.EndEffectorRun;
import frc.robot.commands.EndEffectorScore;
//Elevator Imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorJog;
import frc.robot.commands.ElevatorPlace;
import frc.robot.commands.ElevatorDefault;
//Climb Imports
import frc.robot.subsystems.Climb;
import frc.robot.commands.ClimbRun;


//Limelight Imports
import frc.robot.commands.DrivetrainLeftAlign;
import frc.robot.commands.DrivetrainRightAlign;
import frc.robot.commands.DrivetrainRotAlign;
import frc.robot.commands.DrivetrainRightAlign;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.VisionManager; 

public class RobotContainer {
    //====================GENERAL SETUP====================
    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController driverController = new CommandXboxController(Devices.DRIVER_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(Devices.OPERATOR_CONTROLLER);

    //====================SWERVE SETUP====================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DrivetrainMaxSpeed * 0.1).withRotationalDeadband(Constants.DrivetrainMaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();    

    public RobotContainer() {
        //====================AUTONOMOUS SETUP====================
        //====================Alignment Commands====================
        NamedCommands.registerCommand("DrivetrainLeftAlign", new DrivetrainLeftAlign(drivetrain));
        NamedCommands.registerCommand("DrivetrainRightAlign", new DrivetrainRightAlign(drivetrain));

        //====================Actions====================
        NamedCommands.registerCommand("RobotAutoPrepScoreL4", new RobotAutoPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L4_Setpoint));
        NamedCommands.registerCommand("EndEffectorScore", new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Score_Coral_Speed));
        NamedCommands.registerCommand("GroundIntake", new RobotIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint));

        //====================Zeroing====================
        NamedCommands.registerCommand("RobotHome", new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));
        NamedCommands.registerCommand("EndEffectorStop", new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));
        NamedCommands.registerCommand("GroundIntakeStop", new RobotIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Zero_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("MushroomProcessor");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        //====================DRIVER CONTROLLER BINDINGS====================
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
            .withVelocityX(-1 * MathUtil.applyDeadband(driverController.getLeftY(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
            .withVelocityY(-1 * MathUtil.applyDeadband(driverController.getLeftX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Speed_Multiplier * Constants.DrivetrainMaxSpeed)
            .withRotationalRate(-1 * MathUtil.applyDeadband(driverController.getRightX(), Devices.JOYSTICK_DEADZONE_TOLERANCE) * Constants.Drivetrain_Turn_Multiplier * Constants.DrivetrainMaxAngularRate)
            )
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //====================Swerve Heading Reset====================
        driverController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //====================Align Left====================
        driverController.leftBumper().whileTrue(new DrivetrainLeftAlign(drivetrain));

        //====================Align Right====================
        driverController.rightBumper().whileTrue(new DrivetrainRightAlign(drivetrain));

        //====================Ground Intake====================
        driverController.leftTrigger().whileTrue(new RobotIntakeGround(EndEffector.getInstance(), Constants.End_Effector_Ground_Intake_Speed, Constants.End_Effector_Wrist_Coral_Ground_Setpoint, Intake.getInstance(), Constants.Intake_Ground_Deploy_Setpoint, Constants.Intake_Ground_Run_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Coral_Setpoint));
        driverController.leftTrigger().onFalse(new RobotIntakeGround(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Intake.getInstance(), Constants.Intake_Zero_Setpoint, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Ground Outtake====================
        driverController.povUp().whileTrue(
                Commands.parallel(    
                new IntakeRunCmd(Intake.getInstance(), Constants.Outake_Ground_Run_Speed),
                new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Ground_Outake_Speed)
                )
        );
    
        driverController.povUp().onFalse(
                Commands.parallel(
                new IntakeRunCmd(Intake.getInstance(), Constants.Absolute_Zero),
                new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero)
                )
        );

        //====================End Effector Run====================
        driverController.rightTrigger().whileTrue(new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Score_Coral_Speed));
        driverController.rightTrigger().onFalse(new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));

        //====================Algae Intake====================
        driverController.a().whileTrue(new EndEffectorScore(EndEffector.getInstance(), Constants.End_Effector_Algae_Score_Speed));
        driverController.a().onFalse(new EndEffectorScore(EndEffector.getInstance(), Constants.Absolute_Zero));

        //====================Level 2 Coral Score====================
        driverController.b().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L2_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.b().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 3 Coral Score====================
        driverController.x().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L3_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.x().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Level 4 Coral Score====================
        driverController.y().whileTrue(new RobotPrepScore(EndEffector.getInstance(), Constants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), Constants.Elevator_L4_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.y().onFalse(new RobotHome(EndEffector.getInstance(), Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero));

        //====================Coral Gullet Intake====================
        driverController.button(8).whileTrue(new RobotStationIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Coral_Station_Setpoint, Constants.End_Effector_Coral_Station_Intake_Speed, Elevator.getInstance(), Constants.Elevator_L4_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.button(8).onFalse(new RobotStationIntake(EndEffector.getInstance(), Constants.Absolute_Zero, Constants.Absolute_Zero, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));

        //====================Ground Algae Intake====================
        driverController.rightStick().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Ground_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Ground_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.rightStick().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Bottom Algae Removal====================
        driverController.povLeft().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Bottom_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.povLeft().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Top Algae Removal====================
        driverController.povRight().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Top_Algae_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.povRight().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================Net Algae Score====================
        driverController.leftStick().whileTrue(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Barge_Score_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Elevator_Barge_Score_Setpoint, drivetrain, Constants.Drivetrain_Elevator_Speed_Multiplier, Constants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.leftStick().onFalse(new RobotAlgaeIntake(EndEffector.getInstance(), Constants.End_Effector_Wrist_Algae_Remove_Setpoint, Constants.End_Effector_Algae_Intake_Speed, Elevator.getInstance(), Constants.Absolute_Zero, drivetrain, Constants.Drivetrain_Speed_Multiplier, Constants.Drivetrain_Turn_Multiplier, driverController.getHID()));

        //====================OPERATOR CONTROLLER BINDINGS====================
        //====================Elevator Climb + End Effector=====================
        // operatorController.leftBumper().whileTrue(new RobotPrepScore(EndEffector.getInstance(), PhysConstants., Elevator.getInstance(), PhysConstants., drivetrain, PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier, operatorController.getHID()));
        // operatorController.leftBumper().onFalse(new RobotHome(EndEffector.getInstance(), PhysConstants.Absolute_Zero, Elevator.getInstance(), PhysConstants.Absolute_Zero));

        //====================Climb Up=====================
        // operatorController.rightBumper().whileTrue(new ClimbRun(Climb.getInstance(), Constants.Climb_Up_Speed));
        // operatorController.rightBumper().onFalse(new ClimbRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Climb Down=====================
        // operatorController.leftTrigger().whileTrue(new ClimbRun(Climb.getInstance(), Constants.Climb_Down_Speed));
        // operatorController.leftTrigger().onFalse(new ClimbRun(Climb.getInstance(), Constants.Absolute_Zero));

        //====================Elevator Jog=====================
        operatorController.povUp().whileTrue(new ElevatorJog(Elevator.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));

        //====================End Effector Jog=====================
        operatorController.povRight().whileTrue(new EndEffectorWristJog(EndEffector.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));

        //====================Intake Jog=====================
        operatorController.povLeft().whileTrue(new IntakeWristJog(Intake.getInstance(), () -> operatorController.getRightY() * Devices.JOYSTICK_JOG_SPEED_MULTIPLIER));
    }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }