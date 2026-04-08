package frc.robot;

import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class Constants {
    public static class IntakeWheelConstants {
        public static final double IntakeWheelSpeed = -0.4; // Subject to change
    }
    public static class MotorIDs{
        public static int IntakeAngle = 9;
        public static int IntakeWheels = 10;
        public static int Shooter1 = 15;
        public static int Shooter2 = 16;
        public static int Passthrough = 17;
        public static int Hood = 18;
        public static int Indexer = 19;
        public static int Climber = 20;
    }
    public static class HoodConstants {

        // TODO: Tune hood PID
        public static double kP = 2;
        public static double kI = 0;
        public static double kD = 0;

        public static double kS = 0;
        public static double kG = 0;
        public static double kV = 0;

        // TODO: Tune hood setpoints
        public static double thirtyDegrees = -0.684;
        public static double twentyfiveDegrees = -0.513;
        public static double twentyDegrees = -0.342;
        public static double tenDegrees = 0;
    }

    public static class VisionConstants {
        public static String front = "front";
        public static String side = "side";
        
        public static double rotationPitch = - Math.PI / 9; // 20 degrees up in radians
        public static double translationX = 0.333; // 13.11 in forward in meters
        public static double translationY = 0.006; // 0.25 in left in meters
        public static double translationZ = 0.351; // 13.8 in up in meters

        public static Pose2d blueHub = new Pose2d(4.587, 4.035, new Rotation2d(0));
        public static Pose2d redHub = new Pose2d(16.54 - 4.587, 4.035, new Rotation2d(0));
        
        // Rotation PID not needed
        public static double rotationkP = 1;
        public static double rotationkI = 0;
        public static double rotationkD = 0;

        public static double maxVelocity = 0.3;
        public static double maxAcceleration = 0.3;

        public static double rotationTolerance = Math.PI / 180;
    }

    public static class ShooterConstants{
        public static double shooterTolerance = 0.5; // Tolerance for bang bang controller
        public static double hubHeight = 1.8288; // 6 ft
        public static double shooterHeight = 0.29;
        public static double gearRatio = 0.8333; // 15:18
        public static double circumference = 2 * Math.PI * 0.1016; // 4 in (radius) * 2 * pi
    }
    public static class IntakeConstants {

        // TODO: Tune Intake PID
        public static double kP = 0.75;
        public static double kI = 0;
        public static double kD = 0;

        public static double kS = 0;
        public static double kG = 0;
        public static double kV = 0;
        public static double kA = 0;

        public static double tpV = 0;
        public static double tpA = 0;

        // TODO: Tune Intake PID setpoints
        public static double upPosition = 3.37;
        public static double slighltyUpPosition = 0.5;
        public static double downPosition = 0;
    }
}
