package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePID;
import frc.robot.Constants.IntakeConstants;

public class IntakeUp extends Command{
    private intakePID intake;
    
    public IntakeUp(intakePID i){
        this.intake = i;
        addRequirements(i);
    }

    @Override
    public void execute(){
        intake.set(IntakeConstants.upPosition);
    }

    @Override
    public boolean isFinished(){
        return intake.isAtGoal();
    }
}
