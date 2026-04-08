package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.dnn.Net;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorIDs;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoublePublisher;

public class Shooter extends SubsystemBase {
    TalonFX shooter1;
    TalonFX shooter2;
    BangBangController controller;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("RobotData");
    DoublePublisher shooterSpeedPub;
    
    public Shooter(){
        this.shooter1 = new TalonFX(MotorIDs.Shooter1);
        this.shooter2 = new TalonFX(MotorIDs.Shooter2);

        // Important for BB Controller
        this.shooter1.setNeutralMode(NeutralModeValue.Coast);
        this.shooter2.setNeutralMode(NeutralModeValue.Coast);

        this.controller = new BangBangController(ShooterConstants.shooterTolerance);
        controller.setTolerance(0.5);
        shooterSpeedPub = table.getDoubleTopic("shooterSpeed").publish();
    }

    public void shoot(double velocity){ // Velocity should be output velocity in m/s
        double realVelocity = (velocity * ShooterConstants.gearRatio) / ShooterConstants.circumference; // rotations per second
        System.out.println("real velocity: " + realVelocity);
        this.controller.setSetpoint(realVelocity);
    }
    public void setPercent(double percent){
        this.shooter1.set(percent);
        this.shooter2.set(percent);
    }
    private double getSpeed() {

        return this.shooter1.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed(){
        return this.controller.atSetpoint();
    }

    @Override
    public void periodic(){
        double output = this.controller.calculate(this.getSpeed());
        this.shooter1.set(output);
        this.shooter2.set(output);
        this.shooterSpeedPub.set(this.getSpeed());
    }
}
