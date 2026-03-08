package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorIDs;

public class Shooter extends SubsystemBase {
    TalonFX shooter1;
    TalonFX shooter2;
    BangBangController controller;
    
    public Shooter(){
        this.shooter1 = new TalonFX(MotorIDs.Shooter1);
        this.shooter2 = new TalonFX(MotorIDs.Shooter2);
        this.shooter1.setNeutralMode(NeutralModeValue.Coast);
        this.shooter2.setNeutralMode(NeutralModeValue.Coast);

        this.controller = new BangBangController(ShooterConstants.shooterTolerance);
    }

    // Important: run this continuously with 20ms delay
    public void shoot(double velocity){
        this.shooter1.setVoltage(this.controller.calculate(this.getSpeed(), velocity));
        this.shooter2.setVoltage(this.controller.calculate(this.getSpeed(), velocity));
    }
    private double getSpeed() {
        return this.shooter1.getVelocity().getValueAsDouble();
    }
    public void stop(){
        this.shooter1.set(0);
        this.shooter2.set(0);
    }
}
