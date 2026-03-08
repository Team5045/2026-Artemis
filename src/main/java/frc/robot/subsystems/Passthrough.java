package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.MotorIDs;

public class Passthrough extends SubsystemBase {
    TalonFX passthrough;

    public Passthrough(){
        this.passthrough = new TalonFX(MotorIDs.Passthrough);
    }

    public void run(){
        passthrough.set(0.2);
    }

    public void stop(){
        passthrough.set(0);
    }
}
