package first.robot.subsystems.endEffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.util.Color;

import first.robot.util.PearadoxTalonFX.MotorData;

public interface EEIO {
    @AutoLog
    public static class EEIOInputs {
        public MotorData wristData;
        public MotorData rollerData;

        public Color colorReading;
    }

    public default void updateInputs(EEIOInputs inputs) {}

    public default void setWristAngleDeg(double angleDeg) {}

    public default void setWristAngleDeg(double angleDeg, DoubleSupplier ff) {}

    public default void setRollerVoltage(double volts) {}

    public default Color getColorReading() {return Color.BLACK;}
}
