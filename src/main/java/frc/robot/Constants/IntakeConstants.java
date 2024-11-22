package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeConstants{
    public static final int motorOneCanID = 7;
    public static final int motorTwoCanID = 8;
    public static final MotorType motorOneType = MotorType.kBrushless;
    public static final MotorType motorTwoType = MotorType.kBrushless;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 1;

    public static final int frontBeamBrake = 2;
    public static final int backBeamBrake = 7;
}
