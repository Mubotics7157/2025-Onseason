package frc.robot;

//Drivetrain Imports
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;

//Intake Imports
import frc.robot.subsystems.Intake;
import frc.robot.commands.IntakeRunCmd;

//End Effector Imports
import frc.robot.subsystems.EndEffector;
import frc.robot.commands.EndEffectorRunCmd;
import frc.robot.commands.EndEffectorWristCmd;

//Elevator Imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorJogCmd;
import frc.robot.commands.ElevatorPIDCmd;

//Climb Imports
import frc.robot.subsystems.Climb;
import frc.robot.commands.ClimbRunCmd;

//Limelight Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotContainer {
    //====================GENERAL SETUP====================
    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController DriverController = new CommandXboxController(DeviceConstants.CONTROLLER_DEVICE_ID);

    //====================SWERVE SETUP====================
    private double MaxSpeed = SwerveTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);
    public final Drivetrain drivetrain = SwerveTunerConstants.createDrivetrain();    

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
        drivetrain.registerTelemetry(logger::telemeterize);
    

        //====================SWERVE AUTO LINEUP BINDINGS====================
        //DriveController.leftBumper().whileTrue(new AlignCmd(Drivetrain.getInstance(), PARAMS));

        //====================RIO CANBUS BINDINGS====================
        //Temporary Tai Lung Intake Command Binding
        DriverController.leftTrigger().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        DriverController.leftTrigger().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), 5.0));
        DriverController.leftTrigger().whileTrue(new EndEffectorRunCmd(EndEffector.getInstance(), -0.3));
        // DriverController.leftTrigger().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        // DriverController.leftTrigger().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));
        // DriverController.leftTrigger().onFalse(new EndEffectorRunCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));
        
        //End Effector Run Binding
        DriverController.rightTrigger().whileTrue(new EndEffectorRunCmd(EndEffector.getInstance(), KinematicsConstants.scoreSpeed));

        //Level 1 Sequential Command Binding
        DriverController.a().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), 2.0));
        DriverController.a().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), 0.21));
        DriverController.a().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        DriverController.a().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Level 2 Sequential Command Binding
        DriverController.b().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), 4.0));
        DriverController.b().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), 0.21));
        DriverController.a().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        DriverController.a().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Level 3 Sequential Command Binding
        DriverController.x().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), 6.0));
        DriverController.x().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), 0.21));
        DriverController.a().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        DriverController.a().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Level 4 Sequential Command Binding
        DriverController.y().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), 8.0));
        DriverController.y().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), 0.21));
        DriverController.a().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        DriverController.a().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Elevator Jog Binding
        DriverController.povRight().whileTrue(new ElevatorJogCmd(Elevator.getInstance(), () -> KinematicsConstants.jogSpeedMultiplier * DriverController.getRightY()));

        //Climb Up Binding
        DriverController.povUp().whileTrue(new ClimbRunCmd(Climb.getInstance(), KinematicsConstants.climbUpSpeed));

        //Climb Down Binding
        DriverController.povDown().whileTrue(new ClimbRunCmd(Climb.getInstance(), KinematicsConstants.climbDownSpeed));
        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }