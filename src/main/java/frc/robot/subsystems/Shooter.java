package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
    TalonFX shooter1;
    TalonFX shooter2;
    
    public Shooter(int id1, int id2){
        this.shooter1 = new TalonFX(id1);
        this.shooter2 = new TalonFX(id2);
    }

    public void shoot(){
        // TODO: Bang Bang later with pose estimation
        shooter1.set(0.3);
        shooter2.set(0.3);
    }
    public void stop(){
        shooter1.set(0);
        shooter2.set(0);
    }
}
