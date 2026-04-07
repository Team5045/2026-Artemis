package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

import java.lang.Math;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

// For rumble feedback
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class PrepareShooter extends Command {
    Shooter m_Shooter;
    ShooterHood m_ShooterHood;
    CommandSwerveDrivetrain m_Drivetrain;
    //ProfiledPIDController m_Controller;
    SwerveRequest.FieldCentricFacingAngle drive;
    SwerveRequest.SwerveDriveBrake brake;
    double targetAngle;
    double hoodSetpoint;
    double hoodAngle;
    double velocity;
    CommandXboxController joystick;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("RobotData");
    BooleanPublisher alignedPub;
    BooleanPublisher speedPub;

    public PrepareShooter(Shooter m_Shooter, ShooterHood m_ShooterHood, CommandSwerveDrivetrain m_Drivetrain, SwerveRequest.SwerveDriveBrake brake, CommandXboxController joystick){
        this.m_Shooter = m_Shooter;
        this.m_ShooterHood = m_ShooterHood;
        this.m_Drivetrain = m_Drivetrain;
        //this.m_Controller = new ProfiledPIDController(VisionConstants.rotationkP, VisionConstants.rotationkI, VisionConstants.rotationkD, new TrapezoidProfile.Constraints(VisionConstants.maxVelocity, VisionConstants.maxAcceleration));
        
        double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        this.drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;
        this.brake = brake;
        
        this.targetAngle = 0;
        this.hoodSetpoint = 0;
        this.velocity = 0;

        this.alignedPub = table.getBooleanTopic("aligned").publish();
        this.speedPub = table.getBooleanTopic("atTargetSpeed").publish();

        this.joystick = joystick;

        addRequirements(m_Shooter);
        addRequirements(m_ShooterHood);
        addRequirements(m_Drivetrain);
    }

    @Override
    public void initialize(){
        Pose2d pose = m_Drivetrain.getState().Pose;
        var alliance = DriverStation.getAlliance();
        Pose2d hubPose;
        if(alliance.isPresent() && alliance.get() == Alliance.Red && DriverStation.isAutonomous()){
            hubPose = VisionConstants.redHub;
        } else {
            hubPose = VisionConstants.blueHub;
        }
        double diffX = pose.getX() - hubPose.getX();
        double diffY = pose.getY() - hubPose.getY();
        double distance = Math.sqrt(Math.pow(diffX, 2) + Math.pow(diffY, 2));
        this.targetAngle = Math.atan(diffY/diffX);
        double angle;
        if(distance > 3.6576) { // 12 ft
            this.hoodSetpoint = HoodConstants.thirtyDegrees; 
            angle = Math.PI/2 - Math.PI/6;
        } else if(distance < 3.6576 && distance > 2.4384){ // 8 ft < d < 12 ft
            this.hoodSetpoint = HoodConstants.twentyfiveDegrees;
            angle = Math.PI/2 - (5*Math.PI)/36;
        } else {
            this.hoodSetpoint = HoodConstants.twentyDegrees;
            angle = Math.PI/2 - Math.PI/9;
        }
        this.velocity = Math.sqrt((-9.8*Math.pow(distance, 2))/(2 * Math.pow(Math.cos(angle), 2) * (ShooterConstants.hubHeight - ShooterConstants.shooterHeight - distance*Math.tan(angle))));
        m_ShooterHood.set(this.hoodSetpoint);
    }

    @Override
    public void execute() {
        m_Shooter.shoot(this.velocity);
        Pose2d pose = m_Drivetrain.getState().Pose;

        boolean aligned = Math.abs(pose.getRotation().getRadians() - this.targetAngle) < VisionConstants.rotationTolerance;
        if(!aligned){
            m_Drivetrain.setControl(drive.withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(new Rotation2d(this.targetAngle)));
            this.joystick.setRumble(RumbleType.kLeftRumble, 0);           
        } else {
            m_Drivetrain.setControl(this.brake);
            this.joystick.setRumble(RumbleType.kLeftRumble, 1);
        }
        
        // Publish aligned to networktables
        this.alignedPub.set(aligned);

        // Publish atTargetSpeed to networktables
        this.speedPub.set(this.m_Shooter.isAtSpeed());
        if(this.m_Shooter.isAtSpeed()){
            this.joystick.setRumble(RumbleType.kRightRumble, 1);
        } else {
            this.joystick.setRumble(RumbleType.kRightRumble, 0);
        }
        //SmartDashboard.putData("driveRotPID", this.m_Controller);
    }

    @Override
    public void end(boolean isInterrupted){
        m_Shooter.shoot(0);
    }
}
