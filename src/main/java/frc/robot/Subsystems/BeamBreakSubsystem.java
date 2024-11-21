package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreakSubsystem {
    private final DigitalInput beamBreakFront;
    private final DigitalInput beamBreakBack;

    public BeamBreakSubsystem() {
        beamBreakFront = new DigitalInput(Constants.BeambreakConstants.frontBeambreakChannel);
        beamBreakBack = new DigitalInput(Constants.BeambreakConstants.backBeambreakChannel);
    }

    public boolean frontBeamBreakIsTriggered() {
        return !beamBreakFront.get(); 
    }

    public boolean backBeamBreakIsTriggered() {
        return !beamBreakBack.get(); 
    }
}
