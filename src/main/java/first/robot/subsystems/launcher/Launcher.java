package first.robot.subsystems.launcher;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

import first.robot.subsystems.launcher.LauncherConstants.LauncherStates;

public class Launcher extends Mechanism {
    private final LauncherIO io;

    @AutoLogOutput(key="Mechanisms/Launcher/State") private LauncherStates state = LauncherStates.OFF;
    @AutoLogOutput(key="Mechanisms/Launcher/Scoring State") private LauncherStates scoringState = LauncherStates.SELF_DIRECTING;

    @AutoLogOutput(key="Mechanisms/Launcher/Raw RPS Target") private double rawMeanRPSTarget = 0.0;
    @AutoLogOutput(key="Mechanisms/Launcher/Adjust") private double adjust = 0.0;
    @AutoLogOutput(key="Mechanisms/Launcher/True RPS Target") private double meanRPSTarget = 0.0;

    @AutoLogOutput(key="Mechanisms/Launcher/Error/Minimum Percent") private double minimumErrorPercent = 0.0;

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
        Logger.recordOutput("Mechanisms/Launcher/Mean RPS", getMeanRPS());
        Logger.recordOutput("Mechanisms/Launcher/Mean Surface Speed MPS", getMeanRPS() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        Logger.recordOutput("Mechanisms/Launcher/Error/Raw RPS", meanRPSTarget - getMeanRPS());
        Logger.recordOutput("Mechanisms/Launcher/Error/Percent", Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100);
        // Logger.recordOutput("Mechanisms/Launcher/Error/Minimum Percent", minimumErrorPercent);
    }

    public Command setLauncherRPS(double RPS) {
        if (state == LauncherStates.OFF) {return Command.noRequirements(co -> {}).named("LAUNCHER IS OFF");}
        return run(co -> {
            rawMeanRPSTarget = RPS;
            meanRPSTarget = rawMeanRPSTarget + adjust;
            meanRPSTarget = (Math.abs(meanRPSTarget) < LauncherConstants.FLYWHEEL_MAX_SPEED_RPS ? meanRPSTarget : LauncherConstants.FLYWHEEL_MAX_SPEED_RPS);
            io.setLauncherRPS(meanRPSTarget);
            minimumErrorPercent = Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100;
            while(minimumErrorPercent > 1) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                if ((meanRPSTarget - getMeanRPS()) / meanRPSTarget < minimumErrorPercent) {minimumErrorPercent = Math.abs((meanRPSTarget - getMeanRPS()) / meanRPSTarget) * 100;}
                co.yield();
            }
        }).named("LAUNCHER RPS " + (Math.abs(RPS+adjust) < LauncherConstants.FLYWHEEL_MAX_SPEED_RPS ? RPS+adjust : LauncherConstants.FLYWHEEL_MAX_SPEED_RPS));
    }

    public Command applyState(LauncherStates state) {
        return run(co -> {
            this.state = state;
            if(state == LauncherStates.OFF) {
                rawMeanRPSTarget = 0.0;
                meanRPSTarget = 0.0;
                io.setLauncherRPS(meanRPSTarget);
            }
        }).named("APPLY STATE " + state);
    }

    public Command setScoringState(LauncherStates state) {
        return Command.noRequirements(co -> {this.scoringState = state;}).named("");
    }

    public Command adjustRPS(double by) {
        return Command.noRequirements(co -> {
            while(true) {
                adjust += by;
                co.await(setLauncherRPS(rawMeanRPSTarget));
                co.yield();
            }
        }).named("ADJUST RPS");
    }

    public LauncherStates getState() {
        return state;
    }

    public LauncherStates getScoringState() {
        return scoringState;
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
