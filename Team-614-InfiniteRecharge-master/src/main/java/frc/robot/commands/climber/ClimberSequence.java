package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.climber.*;
import frc.robot.commands.misc.BlinkForATime;
public class ClimberSequence extends CommandGroup {
    public ClimberSequence() {
        addSequential(new RunClimberUp(2500));
        addSequential(new RunClimberDown());
        addSequential(new BrakePistonIn());
    }
}