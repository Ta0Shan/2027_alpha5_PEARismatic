package first.robot.subsystems.endEffector;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import first.robot.util.PearadoxTalonFX;

public abstract class EEIOTalonFX implements EEIO {
    protected final PearadoxTalonFX wrist;
    protected final PearadoxTalonFX roller;

    protected final PositionVoltage wristPositionVoltage;
    protected final VoltageOut rollerVoltageOut;

    public EEIOTalonFX() {
        wrist = new PearadoxTalonFX(EEConstants.WRIST_ID, EEConstants.WRIST_CONFIG());
        roller = new PearadoxTalonFX(EEConstants.ROLLERS_ID, EEConstants.ROLLER_CONFIG());

        wristPositionVoltage = new PositionVoltage(0);
        rollerVoltageOut = new VoltageOut(0);
    }

    public void updateInputs(EEIOInputs inputs) {
        inputs.wristData = wrist.getData();
        inputs.rollerData = roller.getData();
    }

    public void setWristAngleDeg(double angleDeg) {
        double motorSetpoint = Units.degreesToRotations(angleDeg) * EEConstants.WRIST_REDUCTION;
        wrist.setControl(wristPositionVoltage.withPosition(motorSetpoint));
    }
    
    public void setWristAngleDeg(double angleDeg, double ff) {
        double motorSetpoint = Units.degreesToRotations(angleDeg) * EEConstants.WRIST_REDUCTION;
        wrist.setControl(wristPositionVoltage.withPosition(motorSetpoint).withFeedForward(ff));

    }

    public void setRollerVoltage(double voltage) {
        roller.setControl(rollerVoltageOut.withOutput(voltage));
    }
}
