package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.chassis.FollowPath;
import frc.robot.commands.chassis.FollowPathBackwards;
import frc.robot.commands.misc.BlinkForATime;
import frc.robot.commands.shooter.DeliverGoalHigh;
import frc.robot.commands.shooter.ShootForATime;
import frc.robot.commands.shooter.ShootForATimeWSerializer;
import frc.robot.commands.shooter.ShootForATime2;
import frc.robot.commands.chassis.RotateToAngleBackwards;
import frc.robot.commands.shooter.DeliverGoalLow;

public class ShootSequence2 extends CommandGroup {
    public ShootSequence2() {
        // addSequential(new BlinkForATime(3));
        addParallel(new RotateToAngleBackwards(Robot.m_navX, 0));   
        addParallel(new DeliverGoalLow());
        // addSequential(new FollowPathBackwards());
        // addSequential(new BlinkForATime(3));
        // Robot.m_drivetrain.resetPath();
        // Robot.m_drivetrain.addPoint(0, 0);
        // Robot.m_drivetrain.addPoint(-9, 0);
        // Robot.m_drivetrain.generatePath();
        // addSequential(new FollowPath());
    }
}