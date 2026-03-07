package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDs;

public class ShooterHood extends SubsystemBase {
    TalonFX hood;

    PIDController hoodPID;
    ArmFeedforward hoodFF;

    public ShooterHood() {
        this.hood = new TalonFX(MotorIDs.Hood);
        this.hoodPID = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
        this.hoodFF = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV);
    }

    public void set(double setpoint){
        double position = this.hood.getPosition().getValueAsDouble();
        double velocity = this.hood.getVelocity().getValueAsDouble();
        this.hood.setVoltage(this.hoodPID.calculate(position, setpoint) + this.hoodFF.calculate(position, velocity));
    }
    public void resetPosition(){
        this.hood.setPosition(0);
    }
}
