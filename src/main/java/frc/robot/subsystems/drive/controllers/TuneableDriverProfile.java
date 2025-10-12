package frc.robot.subsystems.drive.controllers;

import frc.robot.subsystems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.util.debugging.LoggedTunableNumber;

public class TuneableDriverProfile {
    private LoggedTunableNumber linearScalar;
    private LoggedTunableNumber linearDeadBand;
    private LoggedTunableNumber linearInputsExponent;
    private LoggedTunableNumber rotationScalar;
    private LoggedTunableNumber rotationInputsExponent;
    private LoggedTunableNumber rotationDeadband;
    private LoggedTunableNumber sniperControl;

    public TuneableDriverProfile(
        double defaultLinearScalar, 
        double defaultLinearDeadband, 
        double defaultLinearInputsExponent,
        double defaultRotationScalar,
        double defaultRotationInputExponent,
        double defaultRotationDeadband,
        double defaultSniperControl,
        String key) {
            linearScalar = new LoggedTunableNumber("Drive/Teleop/"+key+"/LinearScalar", defaultLinearScalar);
            linearDeadBand = new LoggedTunableNumber("Drive/Teleop/"+key+"/LinearDeadband", defaultLinearDeadband);
            linearInputsExponent = new LoggedTunableNumber("Drive/Teleop/"+key+"/LinearInputsExponent", defaultLinearInputsExponent);
            rotationScalar = new LoggedTunableNumber("Drive/Teleop/"+key+"/RotationScalar", defaultRotationScalar);
            rotationInputsExponent = new LoggedTunableNumber("Drive/Teleop/"+key+"/RotationInputsExponent", defaultRotationInputExponent);
            rotationDeadband = new LoggedTunableNumber("Drive/Teleop/"+key+"/RotationDeadband", defaultRotationDeadband);
            sniperControl = new LoggedTunableNumber("Drive/Teleop/"+key+"/SniperControl", defaultSniperControl);
    }

    public TuneableDriverProfile(DriverProfiles defaults) {
        this(
            defaults.linearScalar(), 
            defaults.linearDeadband(), 
            defaults.linearExponent(), 
            defaults.rotationalScalar(), 
            defaults.rotationalScalar(), 
            defaults.rotationDeadband(), 
            defaults.sniperScalar(),
            defaults.key());
    }

    public LoggedTunableNumber linearScalar() { return linearScalar; }
    public LoggedTunableNumber linearDeadBand() { return linearDeadBand; }
    public LoggedTunableNumber linearInputsExponent() { return linearInputsExponent; }
    public LoggedTunableNumber rotationScalar() { return rotationScalar; }
    public LoggedTunableNumber rotationInputsExponent() { return rotationInputsExponent; }
    public LoggedTunableNumber rotationDeadband() { return rotationDeadband; }
    public LoggedTunableNumber sniperControl() { return sniperControl; }
}