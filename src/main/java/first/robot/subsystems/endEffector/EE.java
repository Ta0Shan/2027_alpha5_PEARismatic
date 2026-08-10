package first.robot.subsystems.endEffector;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.subsystems.endEffector.EEConstants.WristStates;
import first.robot.subsystems.endEffector.EEConstants.RollerStates;

public class EE extends Mechanism {

    private final EEIO io;

    private final EEIOInputsAutoLogged inputs = new EEIOInputsAutoLogged();

    private WristStates wristState = WristStates.STOWED;

    private RollerStates rollerState = RollerStates.IDLE;

    public EE(EEIO io) {
        this.io = io;
    }

    public void logIO() {
            io.updateInputs(inputs);
            SmartDashboard.putString("Mechanisms/End Effector/State", wristState.name() + " " + rollerState.name());
            SmartDashboard.putNumber("Mechanisms/End Effector/Wrist/Angle Deg", Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION);
            SmartDashboard.putNumber("Mechanisms/End Effector/Wrist/Setpoint Deg", wristState.getAngleDeg());
            SmartDashboard.putNumber("Mechanisms/End Effector/Rollers/Voltage Setpoint", rollerState.getVoltage());
            SmartDashboard.putNumber("Mechanisms/End Effector/Rollers/Further CW RPS", inputs.rollerData.velocity() / EEConstants.FURTHER_CW_ROLLER_REDUCTION);
            SmartDashboard.putNumber("Mechanisms/End Effector/Rollers/Further CCW RPS", inputs.rollerData.velocity() / EEConstants.FURTHER_CCW_ROLLER_REDUCTION);
    }

    public Command applyState(WristStates wristState, RollerStates rollerState) {
        return run(co -> {
            this.wristState = wristState;
            this.rollerState = rollerState;
            io.setWristAngleDeg(wristState.getAngleDeg());
            io.setRollerVoltage(rollerState.getVoltage());
            while((wristState.getAngleDeg() - Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION) / wristState.getAngleDeg() > 0.05) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                co.yield();
            }
        }).named("APPLY STATE " + wristState.name() + " " + rollerState.name());
    }

    public Command applyState(WristStates wristState) {
        return applyState(wristState, rollerState);
    }
    public Command applyState(RollerStates rollerState) {
        return applyState(wristState, rollerState);
    }

    public WristStates getWristState() {
        return wristState;
    }

    public RollerStates getRollerState() {
        return rollerState;
    }
}
