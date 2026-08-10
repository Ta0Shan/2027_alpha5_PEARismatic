package first.robot.subsystems.telescope;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog; // it doesn't work right now

import first.robot.util.PearadoxTalonFX.MotorData;

public interface TelescopeIO {

    @AutoLog
    public static class TelescopeIOInputs {
        public MotorData pivot1Data;
        public MotorData pivot2Data;
        public MotorData pivot3Data;

        public double pivotAbsEncoderPosition;

        public MotorData arm1Data;
        public MotorData arm2Data;

        public MotorData wristData;
        public MotorData rollersData;
    }

    public default void updateInputs(TelescopeIOInputs inputs) {}

    public default void setPivotAngle(double angleDeg) {}

    public default void setPivotAngle(double angleDeg, DoubleSupplier ff) {}

    public default void setArmExtension(boolean isClimbing, double extensionInches) {}

    public default void setArmExtension(boolean isClimbing, double extensionInches, DoubleSupplier ff) {}

    public default void setWristAngle(double angleDeg) {}

    public default void setWristAngle(double angleDeg, DoubleSupplier ff) {}

    public default void setRollerVolts(double volts) {}

}
