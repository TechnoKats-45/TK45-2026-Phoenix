package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Feeder;

public class AutoIntake extends Command 
{
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;

    public AutoIntake(Intake intake, Floor floor, Feeder feeder) 
    {
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;

        addRequirements(intake, floor, feeder);
    }

    @Override
    public void initialize() 
    {
        intake.setAngle(0); 
    }

    @Override
    public void execute() 
    {
        intake.setIntakePercent(.75);
        floor.setFloorPercent(.5);
        feeder.setFeederPercent(-0.25);
    }

    @Override
    public void end(boolean interrupted) 
    {
        //intake.stow();
        intake.stop();
        floor.stop();
        feeder.stop();
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
