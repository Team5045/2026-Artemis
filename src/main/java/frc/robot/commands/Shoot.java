package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.Passthrough;

public class Shoot extends Command {
    indexer m_Indexer;
    Passthrough m_Passthrough;

    public Shoot(indexer m_Indexer, Passthrough m_Passthrough){
        this.m_Indexer = m_Indexer;
        this.m_Passthrough = m_Passthrough;

        addRequirements(m_Indexer);
        addRequirements(m_Passthrough);
    }

    @Override
    public void execute() {
        this.m_Indexer.go();
        this.m_Passthrough.run();
    }

    @Override
    public void end(boolean isInterrupted){
        this.m_Indexer.no();
        this.m_Passthrough.stop();
    }
}
