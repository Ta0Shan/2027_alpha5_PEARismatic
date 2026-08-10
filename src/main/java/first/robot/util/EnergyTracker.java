package first.robot.util;

import org.wpilib.command3.Command;
import org.wpilib.system.RobotController;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class EnergyTracker {
  public enum Subsystem {
    DRIVE,
    STEER,
    TELESCOPE_PIVOT,
    TELESCOPE_EXTENSION,
    EE_PIVOT,
    EE_ROLLERS,
    LAUNCHER,
    UNASSIGNED
  }

  private static double totalChargeConsumedAh = 0.0;
  private static double totalEnergyConsumedWh = 0.0;

  private static Map<Subsystem, Double> subsystemCharges = new HashMap<>();
  private static Map<Subsystem, Double> subsystemEnergies = new HashMap<>();

  public static void reportCurrentUsage(
      double deltaHours, Subsystem subsystem, double... supplyCurrentDrawAmps) {
    double totalAmps = 0.0;
    for (double amp : supplyCurrentDrawAmps) totalAmps += amp;

    if (deltaHours > 0) {
      // 3 600 000 000 microseconds per hour
      double deltaAmpHours = totalAmps * deltaHours;
      double deltaWattHours = deltaAmpHours * RobotController.getBatteryVoltage();

      totalChargeConsumedAh += deltaAmpHours;
      totalEnergyConsumedWh += deltaWattHours;

      subsystemCharges.merge(subsystem, deltaAmpHours, Double::sum);
      subsystemEnergies.merge(subsystem, deltaWattHours, Double::sum);
    }
  }

  public static Command logEnergy() {
    return Command.noRequirements(co -> {
    Logger.recordOutput("EnergyTracker/Total Charge", totalChargeConsumedAh, "amp hours");
    Logger.recordOutput("EnergyTracker/Total Energy", totalEnergyConsumedWh, "watt hours");

    for (var entry : subsystemCharges.entrySet()) {
      Logger.recordOutput(
          "EnergyTracker/Charges/" + entry.getKey().toString(), entry.getValue(), "amps hours");
      co.yield();
    }

    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyTracker/Energies/" + entry.getKey().toString(), entry.getValue(), "watt hours");
      co.yield();
    }
  }).named("ENERGY LOGS");
  }
}
