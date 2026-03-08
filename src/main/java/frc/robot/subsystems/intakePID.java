package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class intakePID extends SubsystemBase{
    TalonFX motor1;
    ProfiledPIDController pid;
    ArmFeedforward ff;

    // NetworkTables for elastic dashboard
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable table = inst.getTable("RobotData");
    final DoublePublisher intakePosPublisher;

    public intakePID(int id){
        this.motor1 = new TalonFX(id);
        this.pid = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, new TrapezoidProfile.Constraints(IntakeConstants.tpV, IntakeConstants.tpA));
        this.ff = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);
        this.intakePosPublisher = table.getDoubleTopic("intakePos").publish();
    }

    public void set(double goal){
        this.pid.setGoal(goal);
    }

    public boolean isAtGoal(){
        return this.pid.atGoal();
    }

    public void resetPosition(){
        this.motor1.setPosition(0);
    }

    public double getPosition(){
        return motor1.getPosition().getValueAsDouble();
    }
    public double getVelocity(){
        return motor1.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic(){
        this.motor1.set(pid.calculate(this.getPosition()) + ff.calculate(this.getPosition(), this.getVelocity()));
        this.intakePosPublisher.set(this.motor1.getPosition().getValueAsDouble());
        SmartDashboard.putData("intake PID", pid);
    }
}
