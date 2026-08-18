package first.robot;

import org.wpilib.framework.RobotBase; // RobotBase throws an error for some reason ill have to look into it

import com.ctre.phoenix6.CANBus;

import first.robot.subsystems.endEffector.EEConstants.RollerStates;
import first.robot.subsystems.endEffector.EEConstants.WristStates;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;

public class Constants {

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode simMode = Mode.SIM;

    // public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final Mode currentMode = simMode;

    public static final CANBus SUPERSTRUCTURE_CAN_BUS = CANBus.systemcore(0);
    public static final double UPDATE_FREQ_HZ = 50;
    public static final double UPDATE_PERIOD_SEC = 1 / UPDATE_FREQ_HZ;

    public static enum SuperstructureStates {
        HOME(TelescopeStates.HOME, WristStates.STOWED, RollerStates.IDLE, 0.0),
        INTAKING(TelescopeStates.DOWN, WristStates.DEPLOYED, RollerStates.FAST_FWD),
        OUTTAKING(TelescopeStates.DOWN, WristStates.DEPLOYED, RollerStates.FAST_REV),
        L1_FRONT(TelescopeStates.L1_FRONT, WristStates.L1_FRONT),
        L1_BACK(TelescopeStates.L1_BACK, WristStates.L1_BACK),
        L2_FRONT(TelescopeStates.L2_FRONT, WristStates.L2_FRONT),
        L2_BACK(TelescopeStates.L2_BACK, WristStates.L2_BACK),
        CLASSIFIER_FRONT(TelescopeStates.CLASSIFIER_FRONT, WristStates.CLASSIFIER_FRONT),
        CLASSIFIER_BACK(TelescopeStates.CLASSIFIER_BACK, WristStates.CLASSIFIER_BACK),
        LAUNCHER(TelescopeStates.LAUNCHER, WristStates.STOWED),
        CLIMB_RAISED(TelescopeStates.CLIMB_RAISED, WristStates.DEPLOYED),
        CLUMB(TelescopeStates.CLUMB, WristStates.DEPLOYED);

        private final TelescopeStates telescopeState;
        private final WristStates wristState;
        private final RollerStates rollerState;
        private final double launcherRPS;

        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState, double launcherRPS) {
            this.telescopeState = telescopeState;
            this.wristState = wristState;
            this.rollerState = rollerState;
            this.launcherRPS = launcherRPS;
        }
        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState) {
            this(telescopeState, wristState, rollerState, 0.0);
        }
        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState) {
            this(telescopeState, wristState, RollerStates.IDLE, 0.0);
        }

        public TelescopeStates getTelescopeState() {return telescopeState;};
        public WristStates getWristState() {return wristState;};
        public RollerStates getRollerState() {return rollerState;};
        public double getLauncherRPS() {return launcherRPS;}
    }
}
