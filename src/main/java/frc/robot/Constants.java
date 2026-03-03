package frc.robot;

import java.lang.Math;
public class Constants {
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
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static double kS = 0;
        public static double kG = 0;
        public static double kV = 0;

    }

    public static class VisionConstants {
        public static String front = "front";
        public static String side = "side";
        
        public static double rotationPitch = - Math.PI / 9; // 20 degrees up in radians
        public static double translationX = 0.333; // 13.11 in forward in meters
        public static double translationY = 0.006; // 0.25 in left in meters
        public static double translationZ = 0.351; // 13.8 in up in meters
    }
}
