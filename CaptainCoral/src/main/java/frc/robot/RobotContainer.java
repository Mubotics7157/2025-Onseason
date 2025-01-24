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
            .withVelocityX(-DriverController.getLeftY() * KinematicsConstants.drivetrainSpeedMultiplier * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-DriverController.getLeftX() * KinematicsConstants.drivetrainSpeedMultiplier * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        DriverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        DriverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        DriverController.povRight().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //Resets Swerve Heading
        drivetrain.registerTelemetry(logger::telemeterize);

        //====================RIO CANBUS BINDINGS====================
        //Intake Sequential Command Binding
        DriverController.leftTrigger().whileTrue(new IntakeRunCmd(Intake.getInstance(), KinematicsConstants.intakeSpeed));
        //DriverController.leftTrigger().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        //DriverController.leftTrigger().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));
        
        //End Effector Run Binding
        DriverController.rightTrigger().whileTrue(new EndEffectorRunCmd(EndEffector.getInstance(), KinematicsConstants.scoreSpeed));

        //Elevator Jog Binding
        DriverController.povLeft().whileTrue(new ElevatorJogCmd(Elevator.getInstance(), () -> KinematicsConstants.jogSpeedMultiplier * DriverController.getRightY()));

        //Level 1 Sequential Command Binding
        DriverController.a().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L1_Setpoint));
        //DriverController.a().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        //DriverController.a().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L1_L2_L3_Setpoint));
        //DriverController.a().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Level 2 Sequential Command Binding
        DriverController.b().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L2_Setpoint));
        //DriverController.b().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        //DriverController.b().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L1_L2_L3_Setpoint));
        //DriverController.b().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Level 3 Sequential Command Binding
        DriverController.x().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L3_Setpoint));
        //DriverController.x().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        //DriverController.x().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L1_L2_L3_Setpoint));
        //DriverController.x().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Level 4 Sequential Command Binding
        DriverController.y().whileTrue(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.Elevator_L4_Setpoint));
        //DriverController.y().onFalse(new ElevatorPIDCmd(Elevator.getInstance(), KinematicsConstants.absoluteZero));
        //DriverController.y().whileTrue(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.End_Effector_Wrist_L4_Setpoint));
        //DriverController.y().onFalse(new EndEffectorWristCmd(EndEffector.getInstance(), KinematicsConstants.absoluteZero));

        //Climb Up Binding
        DriverController.povUp().whileTrue(new ClimbRunCmd(Climb.getInstance(), KinematicsConstants.climbUpSpeed));

        //Climb Down Binding
        DriverController.povDown().whileTrue(new ClimbRunCmd(Climb.getInstance(), KinematicsConstants.climbDownSpeed));
        }

        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }
    }