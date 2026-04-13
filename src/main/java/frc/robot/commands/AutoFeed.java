package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Intake;
public class AutoFeed extends Command 
{
    public static final String INTAKE_AGITATION_ENABLED_KEY = "Auto Feed/Intake Agitation Enabled";
    private static final double INTAKE_AGITATION_HALF_UP_SECONDS = .5;
    private static final double INTAKE_AGITATION_DOWN_SECONDS = 0.5;

    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private double startTimeSeconds;

    public AutoFeed(Intake intake, Floor floor, Feeder feeder) 
    {
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;

        addRequirements(intake, floor, feeder);
    }

    @Override
    public void initialize() 
    {
        startTimeSeconds = Timer.getFPGATimestamp();
        intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN);
        floor.setFloorPercent(Constants.Floor.AUTO_FEED_SPEED);
        feeder.setFeederPercent(Constants.Feeder.AUTO_FEED_SPEED);
    }

    @Override
    public void execute() 
    {
        boolean intakeAgitationEnabled = SmartDashboard.getBoolean(INTAKE_AGITATION_ENABLED_KEY, false);
        if (intakeAgitationEnabled) 
        {
            double elapsedSeconds = Timer.getFPGATimestamp() - startTimeSeconds;
            double cycleSeconds = INTAKE_AGITATION_HALF_UP_SECONDS + INTAKE_AGITATION_DOWN_SECONDS;
            double cycleTimeSeconds = elapsedSeconds % cycleSeconds;
            double agitationTopAngle = (Constants.Intake.PIVOT_ANGLE_DOWN + Constants.Intake.PIVOT_ANGLE_UP_STOWED) / 4.0 * 3;

            if (cycleTimeSeconds < INTAKE_AGITATION_HALF_UP_SECONDS) {
                intake.setAngle(agitationTopAngle);
            } else {
                intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN);
            }
        } else {
            intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN);
        }

        floor.setFloorPercent(Constants.Floor.AUTO_FEED_SPEED);
        feeder.setFeederPercent(.25);  //Constants.Feeder.AUTO_FEED_SPEED
        intake.setIntakePercent(.25);
    }

    @Override
    public void end(boolean interrupted) 
    {
        intake.setAngle(Constants.Intake.PIVOT_ANGLE_DOWN);
        floor.stop();
        feeder.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
