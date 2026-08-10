// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.telescope;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import first.robot.Constants;

/** Add your docs here. */
public class TelescopeConstants {

    public static final CANBus CAN_BUS = Constants.CAN_BUS;

    public static enum TelescopeStates {
        HOME(60.0, 0.0), // 0 deg
        DOWN(0.0, 0.0), // 0 deg
        L1_FRONT(35.0, 0.0), // 35 deg
        L2_FRONT(35.0, 25.0), // 50 deg
        L1_BACK(147.0, 0.0), // 127.5 deg
        L2_BACK(147.0, 24.0), // 127.5 deg
        CLASSIFIER(0.0, 0.0),
        CLIMB_RAISED(90.0, 7.0), // 90 deg
        CLUMB(90.0, -10), // 90 deg, weird number because gearing's swapped atp
        LAUNCHER(0.0, 0.0); // variable

        private final double pivotAngleDeg;
        private final double armExtensionInches;

        private TelescopeStates(double pivotAngleDeg, double armExtensionInches) {
            this.pivotAngleDeg = pivotAngleDeg;
            this.armExtensionInches = armExtensionInches;
        }

        public double getPivotAngleDeg() {return pivotAngleDeg;}
        public double getArmExtensionInches() {return armExtensionInches;}
    }

    public class PivotConstants {

        public static final int PIVOT_1_ID = 21;
        public static final int PIVOT_2_ID = 22;
        public static final int PIVOT_3_ID = 23;

        public static final double REDUCTION = (48.0/12.0) * (48.0/16.0) * (50.0/12.0) * (52.0/20.0);
        
        public static final TalonFXConfiguration PIVOT_CONFIG() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotionMagic.MotionMagicCruiseVelocity = 9999;
            config.MotionMagic.MotionMagicAcceleration = 200;
            config.MotionMagic.MotionMagicJerk = 400;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 60;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 60;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            // kG should depend on pivot angle and arm extension (different CoM)
            config.Slot0.kP = 1.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0; // TODO: tune

            return config;
        }
        // arm is HORIZONTAL to ground and will rotate COUNTERCLOCKWISE
        public static final double STARTING_ANGLE_DEG = 0.0;
        
        public static final int CANCODER_ID = 24;
        public static final double CANCODER_OFFSET = 0.0 - Units.degreesToRotations(STARTING_ANGLE_DEG);

        public static final CANcoderConfiguration CANCODER_CONFIG() {
            CANcoderConfiguration config = new CANcoderConfiguration();

            config.MagnetSensor.MagnetOffset = CANCODER_OFFSET;
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            return config;
        }

    }

    public class ArmConstants {
        public static final int ARM_1_ID = 31;
        public static final int ARM_2_ID = 32;
        
        public static final double EXTENSION_REDUCTION = (48.0/26.0) * (48.0/12.0) * (30.0/28.0) * (50.0/80.0) * (40.0/50.0);
        public static final double EXTENSION_ROTOR_CIRCUMF_INCHES = Math.PI * (26.0 / 20.0);
        
        public static final double CLIMB_REDUCTION = (50.0/80.0) * (40.0/50.0) * (40.0/16.0) * (48.0/12.0) * (30.0/28.0) * (50.0/20.0);
        public static final double CLIMB_ROTOR_CIRCUMF_INCHES = Math.PI * (26.0 / 20.0);
        
        public static final TalonFXConfiguration CONFIG() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotionMagic.MotionMagicCruiseVelocity = 9999;
            config.MotionMagic.MotionMagicAcceleration = 200;
            config.MotionMagic.MotionMagicJerk = 400; // TODO: tune

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 60;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 60;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            // kG should depend on arm angle
            config.Slot0.kP = 1.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0; // TODO: tune

            return config;
        }

        
        public static final double MIN_LENGTH_METERS = Units.inchesToMeters(28.566252);
        public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(25);
        public static final double MAX_LENGTH_METERS = MIN_LENGTH_METERS + MAX_EXTENSION_METERS;

        public static final double MASS_KG = Units.lbsToKilograms(9.7439771);

    }




    public static enum WristStates {
        STOWED(0),
        DEPLOYED(100),
        L1_FRONT(120),
        L1_BACK(60),
        L2_FRONT(120),
        L2_BACK(60),
        CLASSIFIER(0);

        private final double angleDeg;

        private WristStates(double angleDeg) {
            this.angleDeg = angleDeg;
        }

        public double getAngleDeg() {return angleDeg;}
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

    public class EEConstants {
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
}
