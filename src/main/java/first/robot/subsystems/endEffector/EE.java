package first.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.util.Color;

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
            Logger.processInputs("End Effector", inputs);

            Logger.recordOutput("Mechanisms/End Effector/State", wristState.name() + " " + rollerState.name());
            Logger.recordOutput("Mechanisms/End Effector/Crystal Color", hasCrystal() ? crystalColor() : "N/A");

            Logger.recordOutput("Mechanisms/End Effector/Wrist/Angle Deg", getWristAngleDeg());
            Logger.recordOutput("Mechanisms/End Effector/Wrist/Setpoint Deg", wristState.angleDeg);

            Logger.recordOutput("Mechanisms/End Effector/Rollers/Voltage Setpoint", rollerState.voltage);
            Logger.recordOutput("Mechanisms/End Effector/Rollers/RPS", getRollersRPS());
    }

    public Command applyState(WristStates wristState, RollerStates rollerState) {
        return run(co -> {
            Debouncer setpointDebouncer = new Debouncer(0.2);
            this.wristState = wristState;
            this.rollerState = rollerState;
            io.setWristAngleDeg(wristState.angleDeg);
            io.setRollerVoltage(rollerState.voltage);
            while(setpointDebouncer.calculate(
                Math.abs(wristState.angleDeg - Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION) > 0.5)
            ) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                co.yield();
            }
        }).named("EE " + wristState.name() + " " + rollerState.name());
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

    public double getWristAngleDeg() {
        return Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION;
    }

    public double getRollersRPS() {
        return inputs.rollerData.velocity() / EEConstants.ROLLER_REDUCTION;
    }

    @AutoLogOutput(key="Mechanisms/End Effector/Has Crystal")
    public boolean hasCrystal() {
        return inputs.colorReading != Color.BLACK;
    }

    public String crystalColor() {
        // return inputs.colorReading.toHexString();
        switch(inputs.colorReading.toHexString()) {
            case "#00ff00":
                return "GREEN";
            case "#ffff00":
                return "YELLOW";
            case "#ffa500":
                return "ORANGE";
            case "#800080":
                return "PURPLE";
            default:
                return "BLACK";
        }
    }
}
