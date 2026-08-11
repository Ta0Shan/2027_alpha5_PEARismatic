package first.robot.subsystems.launcher;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import first.robot.util.PearadoxTalonFX;

public abstract class LauncherIOTalonFX implements LauncherIO {
    protected final PearadoxTalonFX launcher1;
    protected final PearadoxTalonFX launcher2;

    protected final VelocityVoltage velocityVoltage;
    protected final CoastOut coastOut;


    public LauncherIOTalonFX() {
        launcher1 = new PearadoxTalonFX(LauncherConstants.LAUNCHER_1_ID,
            LauncherConstants.CONFIG(true));

        launcher2 = new PearadoxTalonFX(LauncherConstants.LAUNCHER_2_ID,
            LauncherConstants.CONFIG(false));

        velocityVoltage = new VelocityVoltage(0);
        coastOut = new CoastOut();
    }

    public void updateInputs(LauncherIOInputs inputs) {
        inputs.launcher1Data = launcher1.getData();
        inputs.launcher2Data = launcher2.getData();
    }

    public void setLauncherRPS(double rps) {
        if (rps == 0) {
            launcher1.setControl(coastOut);
            launcher2.setControl(coastOut);
            // launcher1.setControl(velocityVoltage.withVelocity(0));
            // launcher2.setControl(velocityVoltage.withVelocity(0));
        } else {
            double motorSetpoint = rps * LauncherConstants.REDUCTION;

            // induce spin for stable shot
            launcher1.setControl(velocityVoltage.withVelocity(motorSetpoint + (LauncherConstants.RPS_DIFFERENCE / 2)));

            launcher2.setControl(velocityVoltage.withVelocity(motorSetpoint - (LauncherConstants.RPS_DIFFERENCE / 2)));
        }
    }
}
