package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
public class AutoFeed extends Command 
{
    private final Floor floor;
    private final Feeder feeder;
    private final double durationSeconds;
    private double startTimeSeconds;

    public AutoFeed(Floor floor, Feeder feeder, double durationSeconds) 
    {
        this.floor = floor;
        this.feeder = feeder;
        this.durationSeconds = durationSeconds;

        addRequirements(floor, feeder);
    }

    @Override
    public void initialize() 
    {
        startTimeSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        floor.setFloorPercent(.5);
        feeder.setFeederPercent(1);
    }

    @Override
    public void execute() 
    {
        floor.setFloorPercent(.5);
        feeder.setFeederPercent(1);
    }

    @Override
    public void end(boolean interrupted) 
    {
        floor.stop();
        feeder.stop();
    }

    @Override
    public boolean isFinished() 
    {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTimeSeconds >= durationSeconds;
    }
}
