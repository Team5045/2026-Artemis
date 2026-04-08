package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.Constants.HoodConstants;

public class EmergencyPrepareShooter extends Command{
    Shooter m_Shooter;
    ShooterHood m_ShooterHood;
    
    public EmergencyPrepareShooter(Shooter m_Shooter, ShooterHood m_ShooterHood){
        this.m_Shooter = m_Shooter;
        this.m_ShooterHood = m_ShooterHood;
        addRequirements(m_Shooter);
        addRequirements(m_ShooterHood);
    }

    @Override
    public void execute(){
        this.m_Shooter.setPercent(0.65);
        this.m_ShooterHood.set(HoodConstants.twentyDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_Shooter.setPercent(0);
    }
}
