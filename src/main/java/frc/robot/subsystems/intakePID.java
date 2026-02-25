package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.ctre.phoenix6.hardware.TalonFX;

public class intakePID{
    TalonFX motor1;
    TalonFX motor2;
    PIDController pid;
    ArmFeedforward feedforward;

    public intakePID(TalonFX motor1, TalonFX motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.pid = new PIDController(kP, kI, kD);
        this.feedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    public void goDown(){

    }
}
