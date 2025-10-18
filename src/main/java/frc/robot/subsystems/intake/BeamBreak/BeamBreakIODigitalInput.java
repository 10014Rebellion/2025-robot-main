package frc.robot.subsystems.intake.BeamBreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.BeamBreak.BeamBreakConstants.BeamBreakConfig;

public class BeamBreakIODigitalInput implements BeamBreakIO{

    private DigitalInput digitalInput;
    private boolean invert;

    public BeamBreakIODigitalInput(BeamBreakConfig configuration){
        digitalInput = new DigitalInput(configuration.kID());
        invert = configuration.kInvert();
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs){
        // Setting this to true since we don't care about this value
        inputs.isSensorDetected = true;
        inputs.isDetected = isDetected();
    }

    @Override
    public boolean isDetected(){
        if(invert){
            return !digitalInput.get();
        }

        else{
            return digitalInput.get();
        }
    }
    
}
