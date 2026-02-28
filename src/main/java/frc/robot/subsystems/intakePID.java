package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakeConstants;

public class intakePID{
    TalonFX motor1;
    ProfiledPIDController pid;
    ArmFeedforward ff;

    public intakePID(int id){
        this.motor1 = new TalonFX(id);
        this.pid = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, new TrapezoidProfile.Constraints(IntakeConstants.tpV, IntakeConstants.tpA));
        this.ff = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);
    }

    public void set(double setpoint){
        double position = motor1.getPosition().getValueAsDouble();
        double velocity = motor1.getVelocity().getValueAsDouble();
        this.motor1.set(pid.calculate(position, setpoint) + ff.calculate(position, velocity));
    }
}
