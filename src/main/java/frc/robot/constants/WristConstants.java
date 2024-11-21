package frc.robot.constants;

public final class WristConstants {
    public static final int CAN_ID = 18;

    public static final double MAX_VELOCITY = 80.0; // rotations per second
    public static final double MAX_ACCELERATION = 160.0; // rotations per second^2

    public static final double STOW_POSITION = 0.0;

    //Preset Positions
    public static final double SCORING = 45.0;
    public static final double INTAKE_POSITION = -30.0;
    public static final double HOME_POSITION = 0.0;
    
    
    public static final double kP = 4.8;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    public static final double MIN_ANGLE = -45.0;
    public static final double MAX_ANGLE = 90.0;
}