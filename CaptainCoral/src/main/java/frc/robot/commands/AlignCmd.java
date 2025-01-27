package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveTunerConstants;
import frc.robot.subsystems.Drivetrain;

public class AlignCmd extends Command {
    private final Drivetrain drivetrain;
    private final double xSpeed;
    private final double ySpeed;
    private final double rotSpeed;
    
    public AlignCmd(Drivetrain drivetrain) {
        this.drivetrain = SwerveTunerConstants.createDrivetrain();    
        this.xSpeed = drivetrain.limelight_range_proportional();
        this.ySpeed = drivetrain.limelight_range_proportional();
        this.rotSpeed = drivetrain.limelight_aim_proportional();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCmd Started");
    }

    @Override
    public void execute() {    
        //drivetrain.applyRequest(() -> drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed));
        }

    @Override
    public void end(boolean interrupted) {
        //drivetrain.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
        System.out.println("AlignCmd Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}