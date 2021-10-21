// run right
// run left 
package frc.robot.commands.serializer;
 
import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
// run
 
public class RunSerializerB extends Command {
    public double speed;
    public RunSerializerB(double speed) {
        this.speed = speed;
        requires(Robot.m_serializer);
    }
 
    // Called just before this Command runs the first time
    protected void initialize() {
 
        Robot.m_serializer.setSerializerB(0);
 
    }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        // change to actual value after testing
        Robot.m_serializer.setSerializerB(speed);
    //   Robot.m_feeder.set(0.7);
    }
 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }
 
    // Called once after isFinished returns true
    protected void end() {
        Robot.m_serializer.setSerializerB(0);
 
    }
 
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.m_serializer.setSerializerB(0);
    }
}
