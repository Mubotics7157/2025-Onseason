package frc.robot;

//Drivetrain Imports
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;

//Intake Imports
import frc.robot.subsystems.Intake;
import frc.robot.commands.IntakeWristCmd;
import frc.robot.commands.IntakeRunCmd;

//End Effector Imports
import frc.robot.subsystems.EndEffector;
import frc.robot.commands.EndEffectorWristCmd;
import frc.robot.commands.EndEffectorRunCmd;

//Elevator Imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorJogCmd;
import frc.robot.commands.ElevatorPIDCmd;

//Climb Imports
import frc.robot.subsystems.Climb;
import frc.robot.commands.ClimbRunCmd;

//Limelight Imports
import frc.robot.commands.AlignCmd;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotContainer {
    //====================GENERAL SETUP====================
    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController DriverController = new CommandXboxController(DeviceConstants.CONTROLLER_DEVICE_ID);

    //====================SWERVE SETUP====================
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();    

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        //====================SWERVE CANBUS BINDINGS====================
            drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
            .withVelocityX(-1 * MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * KinematicsConstants.drivetrainSpeedMultiplier * MaxSpeed)
            .withVelocityY(-1 * MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * KinematicsConstants.drivetrainSpeedMultiplier * MaxSpeed)
            .withRotationalRate(-1 * MathUtil.applyDeadband(DriverController.getRightX(), 0.05) * MaxAngularRate)
            )
        );

        DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        DriverController.povLeft().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //Resets Swerve Heading

        //====================SWERVE AUTO LINEUP BINDINGS====================
        // DriverController.leftBumper().whileTrue(new AlignCmd(drivetrain));     

        //====================RIO CANBUS BINDINGS====================
        //====================Ground Intake====================
        DriverController.leftBumper().whileTrue(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new IntakeWristCmd(Intake.getInstance(), KinematicsConstants.Intake_Deploy_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );
    
        DriverController.leftBumper().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new IntakeWristCmd(Intake.getInstance(), KinematicsConstants.Intake_Stow_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Gullet Intake====================
        DriverController.rightBumper().whileTrue(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Gullet_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Gullet_Setpoint)
                )
        );

        DriverController.rightBumper().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Intake Run====================
        DriverController.leftTrigger().whileTrue(new IntakeRunCmd(Intake.getInstance(), KinematicsConstants.intakeSpeed));

        //====================End Effector Run====================
        DriverController.rightTrigger().whileTrue(new EndEffectorRunCmd(EndEffector.getInstance(), KinematicsConstants.scoreSpeed));

        //====================Level 1 Coral Score====================
        DriverController.a().whileTrue(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L1_L2_L3_Score_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L1_Setpoint)
                )
        );

        DriverController.a().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Level 2 Coral Score====================
        DriverController.b().whileTrue(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L1_L2_L3_Score_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L2_Setpoint)
                )
        );

        DriverController.b().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Level 3 Coral Score====================
        DriverController.x().whileTrue(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L1_L2_L3_Score_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L3_Setpoint)
                )
        );
    
        DriverController.x().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Level 4 Coral Score====================
        DriverController.y().whileTrue(
            Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L4_Score_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L4_Setpoint)
                )
        );
    
        DriverController.y().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Algae DeScore====================
        DriverController.povUp().whileTrue(
            Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Bottom_Algae_Remove_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Bottom_Algae_Setpoint)
                )
        );
    
        DriverController.povUp().onFalse(
                Commands.parallel(
                new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_Zero_Setpoint),
                new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_Zero_Setpoint)
                )
        );

        //====================Elevator Jog====================
        // DriverController.povRight().whileTrue(new ElevatorJogCmd(Elevator.getInstance(), () -> KinematicsConstants.jogSpeedMultiplier * DriverController.getRightY()));
        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }