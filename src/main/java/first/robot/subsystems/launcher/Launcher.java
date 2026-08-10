package first.robot.subsystems.launcher;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.smartdashboard.SmartDashboard;

public class Launcher extends Mechanism {
    private final LauncherIO io;

    private double meanRPSTarget = 0.0;
    private double minimumRPSError = 0.0;

    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

    public Launcher(LauncherIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Mechanisms/Launcher/Left/RPS Target",
        (meanRPSTarget == 0 ? 0 : ((meanRPSTarget * LauncherConstants.REDUCTION) + (LauncherConstants.RPS_DIFFERENCE / 2)) / LauncherConstants.REDUCTION));
        SmartDashboard.putNumber("Mechanisms/Launcher/Left/RPS", inputs.launcher1Data.velocity());
        SmartDashboard.putNumber("Mechanisms/Launcher/Left/Surface Speed MPS", inputs.launcher1Data.velocity() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);
        
        SmartDashboard.putNumber("Mechanisms/Launcher/Right/RPS Target", 
        (meanRPSTarget == 0 ? 0 : ((meanRPSTarget * LauncherConstants.REDUCTION) - (LauncherConstants.RPS_DIFFERENCE / 2)) / LauncherConstants.REDUCTION));
        SmartDashboard.putNumber("Mechanisms/Launcher/Right/RPS", inputs.launcher2Data.velocity());
        SmartDashboard.putNumber("Mechanisms/Launcher/Right/Surface Speed MPS", inputs.launcher2Data.velocity() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        SmartDashboard.putNumber("Mechanisms/Launcher/Mean/RPS Target", meanRPSTarget);
        SmartDashboard.putNumber("Mechanisms/Launcher/Mean/RPS", getMeanRPS());
        SmartDashboard.putNumber("Mechanisms/Launcher/Mean/Surface Speed MPS", getMeanRPS() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        SmartDashboard.putNumber("Mechanisms/Launcher/Error/Raw RPS", meanRPSTarget - getMeanRPS());
        SmartDashboard.putNumber("Mechanisms/Launcher/Error/Percent", Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100);
        SmartDashboard.putNumber("Mechanisms/Launcher/Error/Minimum", minimumRPSError);
    }    

    public Command setLauncherRPS(double RPS) {
        return run(co -> {
            meanRPSTarget = Math.signum(RPS) * Math.min(Math.abs(RPS/LauncherConstants.REDUCTION), LauncherConstants.MOTOR_MAX_SPEED_RPS - (LauncherConstants.RPS_DIFFERENCE/2)) * LauncherConstants.REDUCTION;
            io.setLauncherRPS(meanRPSTarget);
            minimumRPSError = Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100;
            while(minimumRPSError > 1) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                if ((meanRPSTarget - getMeanRPS()) / meanRPSTarget < minimumRPSError) {minimumRPSError = Math.abs((meanRPSTarget - getMeanRPS()) / meanRPSTarget) * 100;}
                co.yield();
            }
        }).named("LAUNCHER RPS " + Math.signum(RPS) * Math.min(Math.abs(RPS/LauncherConstants.REDUCTION), LauncherConstants.MOTOR_MAX_SPEED_RPS - (LauncherConstants.RPS_DIFFERENCE/2)) * LauncherConstants.REDUCTION);
    }

    public double getMeanRPS() {
        return mean(inputs.launcher1Data.velocity(), inputs.launcher2Data.velocity()) / LauncherConstants.REDUCTION;
    }

    private double mean(double... values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return (sum / (double)values.length);
  }

}
