package first.robot.subsystems.launcher;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Launcher extends Mechanism {
    private final LauncherIO io;

    @AutoLogOutput(key="Mechanisms/Launcher/Mean/RPS Target")
    private double meanRPSTarget = 0.0;

    @AutoLogOutput(key="Mechanisms/Launcher/Error/Minimum Percent")
    private double minimumErrorPercent = 0.0;

    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

    public Launcher(LauncherIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);

        Logger.recordOutput("Mechanisms/Launcher/Left/RPS Target",
        (meanRPSTarget == 0 ? 0 : ((meanRPSTarget * LauncherConstants.REDUCTION) + (LauncherConstants.RPS_DIFFERENCE / 2)) / LauncherConstants.REDUCTION));
        Logger.recordOutput("Mechanisms/Launcher/Left/RPS", inputs.launcher1Data.velocity());
        Logger.recordOutput("Mechanisms/Launcher/Left/Surface Speed MPS", inputs.launcher1Data.velocity() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);
        
        Logger.recordOutput("Mechanisms/Launcher/Right/RPS Target", 
        (meanRPSTarget == 0 ? 0 : ((meanRPSTarget * LauncherConstants.REDUCTION) - (LauncherConstants.RPS_DIFFERENCE / 2)) / LauncherConstants.REDUCTION));
        Logger.recordOutput("Mechanisms/Launcher/Right/RPS", inputs.launcher2Data.velocity());
        Logger.recordOutput("Mechanisms/Launcher/Right/Surface Speed MPS", inputs.launcher2Data.velocity() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        // Logger.recordOutput("Mechanisms/Launcher/Mean/RPS Target", meanRPSTarget);
        Logger.recordOutput("Mechanisms/Launcher/Mean/RPS", getMeanRPS());
        Logger.recordOutput("Mechanisms/Launcher/Mean/Surface Speed MPS", getMeanRPS() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        Logger.recordOutput("Mechanisms/Launcher/Error/Raw RPS", meanRPSTarget - getMeanRPS());
        Logger.recordOutput("Mechanisms/Launcher/Error/Percent", Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100);
        // Logger.recordOutput("Mechanisms/Launcher/Error/Minimum Percent", minimumErrorPercent);
    }    

    public Command setLauncherRPS(double RPS) {
        return run(co -> {
            meanRPSTarget = (Math.abs(RPS) < LauncherConstants.FLYWHEEL_MAX_SPEED_RPS ? RPS : LauncherConstants.FLYWHEEL_MAX_SPEED_RPS);
            io.setLauncherRPS(meanRPSTarget);
            minimumErrorPercent = Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100;
            while(minimumErrorPercent > 1) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                if ((meanRPSTarget - getMeanRPS()) / meanRPSTarget < minimumErrorPercent) {minimumErrorPercent = Math.abs((meanRPSTarget - getMeanRPS()) / meanRPSTarget) * 100;}
                co.yield();
            }
        }).named("LAUNCHER RPS " + (Math.abs(RPS) < LauncherConstants.FLYWHEEL_MAX_SPEED_RPS ? RPS : LauncherConstants.FLYWHEEL_MAX_SPEED_RPS));
    }

    public double getMeanRPS() {
        return ((inputs.launcher1Data.velocity() + inputs.launcher2Data.velocity()) / 2) / LauncherConstants.REDUCTION;
    }

    private double mean(double... values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return (sum / values.length);
  }

}
