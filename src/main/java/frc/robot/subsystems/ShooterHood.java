package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDs;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class ShooterHood extends SubsystemBase {
    TalonFX hood;

    PIDController hoodPID;
    ArmFeedforward hoodFF;

    // NetworkTables for elastic dashboard
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable table = inst.getTable("RobotData");
    final DoublePublisher hoodPosPublisher;

    public ShooterHood() {
        this.hood = new TalonFX(MotorIDs.Hood);
        this.hoodPID = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
        this.hoodFF = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV);
        this.hoodPosPublisher = table.getDoubleTopic("hoodPos").publish();
    }

    public void set(double setpoint){
        this.hoodPID.setSetpoint(setpoint);
    }
    public void resetPosition(){
        this.hood.setPosition(0);
    }

    public double getPosition(){
        return hood.getPosition().getValueAsDouble();
    }
    public double getVelocity(){
        return hood.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic(){
        this.hood.setVoltage(this.hoodPID.calculate(this.getPosition()) + this.hoodFF.calculate(this.getPosition(), this.getVelocity()));
        this.hoodPosPublisher.set(this.hood.getPosition().getValueAsDouble());
    }
}
