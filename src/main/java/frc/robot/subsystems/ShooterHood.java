package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.HoodConstants;

public class ShooterHood extends SubsystemBase {
    TalonFX hood;

    PIDController hoodPID;
    ArmFeedforward hoodFF;

    public ShooterHood(int id) {
        this.hood = new TalonFX(id);
        this.hoodPID = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
        this.hoodFF = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV);
    }

    public void set(double setpoint){
        double position = this.hood.getPosition().getValueAsDouble();
        double velocity = this.hood.getVelocity().getValueAsDouble();
        this.hood.set(this.hoodPID.calculate(position, setpoint) + this.hoodFF.calculate(position, velocity));
    }
}
