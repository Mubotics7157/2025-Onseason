package frc.robot;

//Drivetrain Imports
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
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

//End Effector Imports
import frc.robot.subsystems.EndEffector;
import frc.robot.commands.EndEffectorWrist;
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
    private final CommandXboxController driverController = new CommandXboxController(DeviceConstants.DRIVER_CONTROLLER_DEVICE_ID);
    private final CommandXboxController operatorController = new CommandXboxController(DeviceConstants.OPERATOR_CONTROLLER_DEVICE_ID);

    //====================SWERVE SETUP====================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(PhysConstants.DrivetrainMaxSpeed * 0.1).withRotationalDeadband(PhysConstants.DrivetrainMaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();    

    public RobotContainer() {
        //====================AUTONOMOUS SETUP====================
        //Alignment Commands
        NamedCommands.registerCommand("leftAlignCmd", new DrivetrainLeftAlign(drivetrain));
        NamedCommands.registerCommand("rightAlignCmd", new DrivetrainRightAlign(drivetrain));

        //Zeroing Commands
        NamedCommands.registerCommand("zeroEndEffectorWrist", new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_Zero_Setpoint));
        NamedCommands.registerCommand("zeroElevator", new ElevatorDefault(Elevator.getInstance(), PhysConstants.Elevator_Zero_Setpoint));
        NamedCommands.registerCommand("endEffectorStop", new EndEffectorScore(EndEffector.getInstance(), PhysConstants.Absolute_Zero));
        NamedCommands.registerCommand("stopGroundIntake", new RobotIntakeGround(EndEffector.getInstance(), 0.0, 0.0, Intake.getInstance(), 0.0, 0.0, Elevator.getInstance(), 0.0));

        //Action Commands
        NamedCommands.registerCommand("endEffectorL4", new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_L4_Score_Setpoint));
        NamedCommands.registerCommand("elevatorL4", new ElevatorDefault(Elevator.getInstance(), PhysConstants.Elevator_L4_Setpoint));
        NamedCommands.registerCommand("endEffectorScore", new EndEffectorScore(EndEffector.getInstance(), PhysConstants.End_Effector_Score_Speed));
        NamedCommands.registerCommand("groundIntake", new RobotIntakeGround(EndEffector.getInstance(), 0.375, 0.573, Intake.getInstance(), 17.0, -0.8, Elevator.getInstance(), 0.5));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("Middle");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        //====================DRIVER CONTROLLER BINDINGS====================
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
            .withVelocityX(-1 * MathUtil.applyDeadband(driverController.getLeftY(), 0.05) * PhysConstants.Drivetrain_Speed_Multiplier * PhysConstants.DrivetrainMaxSpeed)
            .withVelocityY(-1 * MathUtil.applyDeadband(driverController.getLeftX(), 0.05) * PhysConstants.Drivetrain_Speed_Multiplier * PhysConstants.DrivetrainMaxSpeed)
            .withRotationalRate(-1 * MathUtil.applyDeadband(driverController.getRightX(), 0.05) * PhysConstants.Drivetrain_Turn_Multiplier * PhysConstants.DrivetrainMaxAngularRate)
            )
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //====================Swerve Heading Reset====================
        driverController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //====================Align Left====================
        driverController.povLeft().whileTrue(new DrivetrainLeftAlign(drivetrain));

        //====================Align Right====================
        driverController.povRight().whileTrue(new DrivetrainRightAlign(drivetrain));

        //====================Ground Intake====================
        driverController.leftTrigger().whileTrue(new RobotIntakeGround(EndEffector.getInstance(), 0.375, 0.573, Intake.getInstance(), 17.0, -0.8, Elevator.getInstance(), 0.5));
        driverController.leftTrigger().onFalse(new RobotIntakeGround(EndEffector.getInstance(), 0.0, 0.0, Intake.getInstance(), 0.0, 0.0, Elevator.getInstance(), 0.0));

        //====================Ground Outtake====================
        driverController.povUp().whileTrue(
                Commands.parallel(    
                new IntakeRunCmd(Intake.getInstance(), PhysConstants.Outake_Ground_Run_Speed),
                new EndEffectorRun(EndEffector.getInstance(), PhysConstants.End_Effector_Ground_Outake_Speed, driverController.getHID())
                )
        );
    
        driverController.povUp().onFalse(
                Commands.parallel(
                new IntakeRunCmd(Intake.getInstance(), PhysConstants.Absolute_Zero),
                new EndEffectorRun(EndEffector.getInstance(), PhysConstants.Absolute_Zero, driverController.getHID())
                )
        );

        //====================End Effector Score====================
        driverController.rightTrigger().whileTrue(new EndEffectorScore(EndEffector.getInstance(), PhysConstants.End_Effector_Score_Speed));
        driverController.rightTrigger().onFalse(new EndEffectorScore(EndEffector.getInstance(), PhysConstants.Absolute_Zero));

        //====================Level 1 Coral Score====================
        driverController.a().whileTrue(new EndEffectorScore(EndEffector.getInstance(), 0.2));
        driverController.a().onFalse(new EndEffectorScore(EndEffector.getInstance(), PhysConstants.Absolute_Zero));

        //====================Level 2 Coral Score====================
        // DriverController.b().whileTrue(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L2_L3_Score_Setpoint),
        //         new ElevatorPlace(Elevator.getInstance(), KinematicsConstants.Elevator_L2_Setpoint, drivetrain, DriverController.getHID(), KinematicsConstants.Drivetrain_Elevator_Speed_Multiplier, KinematicsConstants.Drivetrain_Elevator_Turn_Multiplier)
        //         )
        // );

        // DriverController.b().onFalse(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
        //         new ElevatorDefault(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
        //         )
        // );

        driverController.b().whileTrue(new RobotPrepScore(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), PhysConstants.Elevator_L2_Setpoint, drivetrain, PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.b().onFalse(new RobotHome(EndEffector.getInstance(), PhysConstants.Absolute_Zero, Elevator.getInstance(), PhysConstants.Absolute_Zero));

        //====================Level 3 Coral Score====================
        // DriverController.x().whileTrue(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_L2_L3_Score_Setpoint),
        //         new ElevatorPlace(Elevator.getInstance(), PhysConstants.Elevator_L3_Setpoint, drivetrain, DriverController.getHID(), PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier)
        //         )
        // );
    
        // DriverController.x().onFalse(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_Zero_Setpoint),
        //         new ElevatorDefault(Elevator.getInstance(), PhysConstants.Elevator_Zero_Setpoint)
        //         )
        // );

        driverController.x().whileTrue(new RobotPrepScore(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_L2_L3_Score_Setpoint, Elevator.getInstance(), PhysConstants.Elevator_L3_Setpoint, drivetrain, PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.x().onFalse(new RobotHome(EndEffector.getInstance(), PhysConstants.Absolute_Zero, Elevator.getInstance(), PhysConstants.Absolute_Zero));

        //====================Level 4 Coral Score====================
        // DriverController.x().whileTrue(
        //     Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_L4_Score_Setpoint),
        //         new ElevatorPlace(Elevator.getInstance(), PhysConstants.Elevator_L4_Setpoint, drivetrain, DriverController.getHID(), PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier)
        //         )
        // );
    
        // DriverController.y().onFalse(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_Zero_Setpoint),
        //         new ElevatorDefault(Elevator.getInstance(), PhysConstants.Elevator_Zero_Setpoint)
        //         )
        // );

        driverController.y().whileTrue(new RobotPrepScore(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_L4_Score_Setpoint, Elevator.getInstance(), PhysConstants.Elevator_L4_Setpoint, drivetrain, PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.y().onFalse(new RobotHome(EndEffector.getInstance(), PhysConstants.Absolute_Zero, Elevator.getInstance(), PhysConstants.Absolute_Zero));

        //====================Gullet Intake====================
        // DriverController.button(8).whileTrue(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_Coral_Station_Setpoint),
        //         new ElevatorPlace(Elevator.getInstance(), PhysConstants.Elevator_Coral_Station_Setpoint, drivetrain, DriverController.getHID(), PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier)
        //         )
        // );

        // DriverController.button(8).onFalse(
        //         Commands.parallel(
        //         new EndEffectorWrist(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_Zero_Setpoint),
        //         new ElevatorDefault(Elevator.getInstance(), PhysConstants.Elevator_Zero_Setpoint)
        //         )
        // );

        driverController.button(8).whileTrue(new RobotStationIntake(EndEffector.getInstance(), PhysConstants.End_Effector_Wrist_Coral_Station_Setpoint, PhysConstants.End_Effector_Coral_Station_Intake_Speed, Elevator.getInstance(), PhysConstants.Elevator_L4_Setpoint, drivetrain, PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));
        driverController.button(8).onFalse(new RobotStationIntake(EndEffector.getInstance(), PhysConstants.Absolute_Zero, PhysConstants.Absolute_Zero, Elevator.getInstance(), PhysConstants.Absolute_Zero, drivetrain, PhysConstants.Drivetrain_Elevator_Speed_Multiplier, PhysConstants.Drivetrain_Elevator_Turn_Multiplier, driverController.getHID()));

        //====================OPERATOR CONTROLLER BINDINGS====================

        //====================Elevator Jog=====================
        //DriverController.povRight().whileTrue(new ElevatorJog(Elevator.getInstance(), () -> KinematicsConstants.Jog_Speed_Multiplier * DriverController.getRightY()));
        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }