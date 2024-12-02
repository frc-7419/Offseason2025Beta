package frc.robot.constants;

public final class WristConstants {
    public static final int canID = 18;

    public static final double maxVelocity = 80.0; // rotations per second
    public static final double maxAcceleration = 160.0; // rotations per second^2

    public static final double stowPosition = 0.0;

    //Preset Positions
    public static final double scoring = 45.0;
    public static final double intakePosition = -30.0;
    public static final double homePosition = 0.0;
    
    
    public static final double kP = 4.8;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    public static final double minAngle = -45.0;
    public static final double maxAngle = 90.0;
}