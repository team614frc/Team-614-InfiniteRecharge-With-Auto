package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.chassis.FollowPath;
import frc.robot.commands.chassis.FollowPathBackwards;
import frc.robot.commands.misc.BlinkForATime;
import frc.robot.commands.shooter.ShootForATime;
import frc.robot.commands.shooter.ShootForATimeWSerializer;
import frc.robot.commands.shooter.ShootForATime2;
import frc.robot.commands.intake.IntakeToggle;
import frc.robot.commands.chassis.FollowPath2;
import frc.robot.commands.chassis.FollowPathBackwards2;
import frc.robot.commands.chassis.FollowPath3;
import frc.robot.commands.chassis.FollowPath3Backwards;
import frc.robot.commands.shooter.ShootForATimeWSerializerFake;
public class LeftPath extends CommandGroup {
    public LeftPath() {
        //6 ball auto below
        addSequential(new ShootForATime(2));

        addSequential(new FollowPath());    
        addSequential(new FollowPath2());

        addSequential(new IntakeToggle());
        addSequential(new ShootForATimeWSerializer(3));
     
        //5 Ball Auto
        // addSequential(new FollowPath());    
        // addSequential(new FollowPath2());
        // addSequential(new ShootForATimeWSerializer(3)); 
        // addSequential(new FollowPath3());
        // addSequential(new FollowPath3Backwards());  
        // addSequential(new ShootForATimeWSerializer(3));  
    }
}