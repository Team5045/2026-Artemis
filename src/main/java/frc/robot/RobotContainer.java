// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intakePID;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.IntakeJiggle;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.Constants.MotorIDs;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.Passthrough;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.Shoot;

public class RobotContainer {
    // idk
    private final IntakeWheels m_IntakeWheels = new IntakeWheels(10);
    private final CommandXboxController m_driverController =
        new CommandXboxController(0);
    private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeWheels, m_driverController);
    

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick1 = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final intakePID intakePID = new intakePID(9);
    public final IntakeDown intakeDown = new IntakeDown(intakePID);
    public final IntakeUp intakeUp = new IntakeUp(intakePID);
    public final IntakeJiggle intakeJiggle = new IntakeJiggle(intakePID);
    public final Shooter m_Shooter = new Shooter();
    public final ShooterHood m_ShooterHood = new ShooterHood();
    public final PrepareShooter m_PrepareShooter = new PrepareShooter(m_Shooter, m_ShooterHood, drivetrain, drive, brake);

    public final Passthrough m_Passthrough = new Passthrough();
    public final indexer m_Indexer = new indexer(MotorIDs.Indexer);
    public final Shoot m_Shoot = new Shoot(m_Indexer, m_Passthrough);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick1.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Intake PID
        joystick2.a().onTrue(intakeDown);
        joystick2.y().onTrue(intakeUp);
        joystick2.b().onTrue(intakeJiggle);


        // Intake wheels
        m_driverController.x().toggleOnTrue(m_IntakeCommand);

        drivetrain.registerTelemetry(logger::telemeterize);

        // Shooter
        joystick1.leftTrigger(0.1).onTrue(m_PrepareShooter);
        joystick1.rightTrigger(0.1).onTrue(m_Shoot);
        
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
