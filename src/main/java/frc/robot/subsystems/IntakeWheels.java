package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeWheels extends SubsystemBase{
    TalonFX motor;

    public IntakeWheels(int motorID) {
        this.motor = new TalonFX(motorID);
    }

    public void setMotorSpeeds(double speed) {
        this.motor.set(speed);
    }
}
