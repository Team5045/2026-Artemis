package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class indexer extends SubsystemBase {
    TalonFX index1;
    
    
    public indexer(int id19){
        this.index1 = new TalonFX(id19);

    }

    public void go(){
        //code that meowgicly works
        index1.set(0.3);
    }
    public void no(){
        index1.set(0);
    }
}
