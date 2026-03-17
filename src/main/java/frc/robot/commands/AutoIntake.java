package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command 
{
    private final Intake intake;
    private final Floor floor;

    public AutoIntake(Intake intake, Floor floor) 
    {
        this.intake = intake;
        this.floor = floor;

        addRequirements(intake, floor);
    }

    @Override
    public void initialize() 
    {
        intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN);
    }

    @Override
    public void execute() 
    {
        intake.setIntakePercent(Constants.Intake.INTAKE_SPEED);
        floor.setFloorPercent(Constants.Floor.FLOOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) 
    {
        intake.stow();
        intake.stop();
        floor.stop();
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
