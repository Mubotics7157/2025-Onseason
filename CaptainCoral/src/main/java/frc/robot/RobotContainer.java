package frc.robot;

//Drivetrain Imports
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;

//Intake Imports
import frc.robot.subsystems.Intake;

//End Effector Imports
import frc.robot.subsystems.EndEffector;
import frc.robot.commands.EndEffectorRunCmd;

//Elevator Imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorJogCmd;
import frc.robot.commands.ElevatorPIDCmd;

//Climb Imports
import frc.robot.subsystems.Climb;

public class RobotContainer {
    private double MaxSpeed = SwerveTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    private final CommandXboxController DriverController = new CommandXboxController(0);

    public final Drivetrain drivetrain = SwerveTunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
            .withVelocityX(-DriverController.getLeftY() * 0.3 * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-DriverController.getLeftX() * 0.3 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))
        ));

        DriverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        DriverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //Swerve Troubleshooting
        DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //Resets Swerve Heading
        drivetrain.registerTelemetry(logger::telemeterize);

        //Elevator Jog Command
        DriverController.povUp().whileTrue(new ElevatorJogCmd(Elevator.getInstance(), () -> KinematicsConstants.jogSpeedMultiplier * DriverController.getRightY()));

    //Level 1
    DriverController.a().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), 3.33));
    // driverController.a().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), 0.13));
    // driverController.a().whileTrue(new EndEffectorWrist(EndEffector.getInstance(), 0.21));
    // driverController.a().onFalse(new EndEffectorWrist(EndEffector.getInstance(), 0.0));

    // //Level 2
    // driverController.b().whileTrue(new ElevatorProfiledPID(ElevatorSubsystem.getInstance(), 2.5));
    // driverController.b().onFalse(new ElevatorProfiledPID(ElevatorSubsystem.getInstance(), 0.0));
    // driverController.b().whileTrue(new EndEffectorWrist(EndEffectorWristSubsystem.getInstance(), 0.21));
    // driverController.b().onFalse(new EndEffectorWrist(EndEffectorWristSubsystem.getInstance(), 0.0));

    // //Level 3
    // driverController.x().whileTrue(new ElevatorProfiledPID(ElevatorSubsystem.getInstance(), 4.60));
    // driverController.x().onFalse(new ElevatorProfiledPID(ElevatorSubsystem.getInstance(), 0.0));
    // driverController.x().whileTrue(new EndEffectorWrist(EndEffectorWristSubsystem.getInstance(), 0.21));
    // driverController.x().onFalse(new EndEffectorWrist(EndEffectorWristSubsystem.getInstance(), 0.0));

    // //Level 4
    // driverController.y().whileTrue(new ElevatorProfiledPID(ElevatorSubsystem.getInstance(), 6.80));
    // driverController.y().onFalse(new ElevatorProfiledPID(ElevatorSubsystem.getInstance(), 0.0));
    // driverController.y().whileTrue(new EndEffectorWrist(EndEffectorWristSubsystem.getInstance(), 0.21));
    // driverController.y().onFalse(new EndEffectorWrist(EndEffectorWristSubsystem.getInstance(), 0.0));

    //End Effector Run
    DriverController.rightTrigger().whileTrue(new EndEffectorRunCmd(EndEffector.getInstance(), 0.3));
    // driverController.button(DeviceConstants.INTAKE_BUTTON).whileTrue(new Score(EndEffectorSubsystem.getInstance(), -0.3));
    }

    //Intake Run
    //DriverController.leftBumper().whileTrue(new EndEffectorRunCmd(EndEffector.getInstance(), 0.3));
    // driverController.button(DeviceConstants.INTAKE_BUTTON).whileTrue(new Score(EndEffectorSubsystem.getInstance(), -0.3));
    //}

    //Runs auto path selected from chooser
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}