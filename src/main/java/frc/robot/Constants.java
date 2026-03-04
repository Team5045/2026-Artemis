package frc.robot;

public class Constants {
    public static class IntakeWheelConstants {
        public static final double IntakeWheelSpeed = 0.4; // Subject to change
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
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static double kS = 0;
        public static double kG = 0;
        public static double kV = 0;

    }
    public static class IntakeConstants {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static double kS = 0;
        public static double kG = 0;
        public static double kV = 0;
        public static double kA = 0;

        public static double tpV = 0;
        public static double tpA = 0;

        public static double upPosition = 0;
        public static double slighltyUpPosition = 0;
        public static double downPosition = 0;
    }
}
