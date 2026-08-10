package first.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import first.robot.util.PearadoxTalonFX.MotorData;

public interface LauncherIO {

    @AutoLog
    public static class LauncherIOInputs {
        public MotorData launcher1Data;
        public MotorData launcher2Data;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}

    public default void setLauncherRPS(double rps) {}

}
