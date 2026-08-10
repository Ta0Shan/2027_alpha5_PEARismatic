package first.robot.subsystems.endEffector;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.Constants;

public class EEConstants {

    public static enum WristStates {
        STOWED(0),
        DEPLOYED(100),
        L1_FRONT(120),
        L1_BACK(60),
        L2_FRONT(120),
        L2_BACK(60),
        CLASSIFIER(0);

        private final double angleFromArmDeg;

        private WristStates(double angleFromArmDeg) {
            this.angleFromArmDeg = angleFromArmDeg;
        }

        public double getAngleDeg() {
            return angleFromArmDeg;
        }
    }

    public static enum RollerStates {
        IDLE(0),
        FWD(5),
        REV(-5),
        FAST_FWD(10),
        FAST_REV(-10);

        private final double voltage;

        private RollerStates(double voltage) {
            this.voltage = voltage;
        }

        public double getVoltage() {return voltage;}
    }

    public static final CANBus CAN_BUS = Constants.CAN_BUS;

    public static final int WRIST_ID = 51;
    public static final int ROLLERS_ID = 52;

    public static final double WRIST_REDUCTION = (36.0 / 8.0) * (38.0/14.0) * (36.0/14.0) * (20.0/16.0);
    public static final double FURTHER_CW_ROLLER_REDUCTION = (32.0/16.0) * (30.0/24.0);
    public static final double FURTHER_CCW_ROLLER_REDUCTION = (32.0/16.0) * (36.0/24.0) * (30.0/30.0); // unused bc voltage control

    public static final TalonFXConfiguration WRIST_CONFIG() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 1.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0; // TODO: tune

        return config;
    }

    public static final TalonFXConfiguration ROLLER_CONFIG() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0; // TODO: tune if you want to use VelocityVoltage

        return config;
    }
    public static final double LENGTH_METERS = Units.inchesToMeters(12.689063);
    public static final double MASS_KG = Units.lbsToKilograms(10); // some parts don't have defined weight, +0.3 lbs to be safe

}
