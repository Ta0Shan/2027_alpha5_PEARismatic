package first.robot.util;

import org.wpilib.system.RobotController;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import first.robot.Constants;
import first.robot.util.EnergyTracker.Subsystem;

/**
 * PearadoxTalonFX is a wrapper around the CTRE TalonFX motor controller. It applies configuration,
 * optimizes CAN bus usage, and provides structured access to telemetry.
 */
public class PearadoxTalonFX extends TalonFX {
  private final BaseStatusSignal[] telemetrySignals;

  private double lastTimestamp;

  private Subsystem subsystem;

  /**
   * Constructs a new PearadoxTalonFX with the specified device ID, CAN Bus, and configuration.
   *
   * @param deviceId CAN device ID for the TalonFX
   * @param canBus CAN Bus that this device is on
   * @param config TalonFXConfiguration to apply
   * @param subsystem the subsystem that this device is a part of
   */
  public PearadoxTalonFX(int deviceId, CANBus canBus, TalonFXConfiguration config, Subsystem subsystem) {
    super(deviceId, canBus);
    applyConfig(config);

    telemetrySignals =
        new BaseStatusSignal[] {
          getPosition(false),
          getVelocity(false),
          getAcceleration(false),
          getMotorVoltage(false),
          getSupplyCurrent(false),
          getStatorCurrent(false),
          getTorqueCurrent(false),
          getDeviceTemp(false)
        };

    BaseStatusSignal.setUpdateFrequencyForAll(Constants.LOOP_FREQ_HZ, telemetrySignals);

    this.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, telemetrySignals);

    this.lastTimestamp = RobotController.getTime();

    this.subsystem = subsystem;
  }


    /**
     * Constructs a new PearadoxTalonFX with the specified device ID and configuration.
     * <p> The energy consumption of this device will not be logged under a specific subsystem.
     * 
     * @param deviceId CAN device ID for the TalonFX
     * @param canBus CAN Bus that this device is on
     * @param config TalonFXConfiguration to apply
     */
    public PearadoxTalonFX(int deviceId, CANBus canBus, TalonFXConfiguration config) {
      this(deviceId, canBus, config, Subsystem.UNASSIGNED);
    }


  /**
   * Applies the given configuration to this motor controller, retrying up to 5 times in case of
   * transient failures.
   *
   * @param config TalonFXConfiguration to apply
   */
  public void applyConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(5, () -> this.getConfigurator().apply(config, 0.25));
  }

  /**
   * Retrieves key telemetry from this motor, including a connectivity flag.
   *
   * @return MotorData with position, velocity, voltage, currents, temperature, and isConnected
   */
  public MotorData getData() {
    boolean connected = BaseStatusSignal.isAllGood(telemetrySignals);

    double supplyCurrent = telemetrySignals[4].getValueAsDouble();

    double newTimestamp = RobotController.getTime();
    double deltaHours = (newTimestamp - lastTimestamp) / 3.6e9;
    lastTimestamp = newTimestamp;

    EnergyTracker.reportCurrentUsage(deltaHours, subsystem, supplyCurrent);

    return new MotorData(
        telemetrySignals[0].getValueAsDouble(), // position
        telemetrySignals[1].getValueAsDouble(), // velocity
        telemetrySignals[2].getValueAsDouble(), // acceleration
        telemetrySignals[3].getValueAsDouble(), // voltage
        supplyCurrent,
        telemetrySignals[5].getValueAsDouble(), // stator current
        telemetrySignals[6].getValueAsDouble(), // torque current
        telemetrySignals[7].getValueAsDouble(), // temperature
        connected);
  }

  /**
   * Record representing a snapshot of TalonFX telemetry.
   *
   * @param position Sensor position (rotations)
   * @param velocity Sensor velocity (rotations/sec)
   * @param acceleration Sensor acceleration (rotations/sec^2) (will not work in sim)
   * @param appliedVolts Voltage applied to motor
   * @param supplyCurrent Current drawn from supply (A)
   * @param statorCurrent Current through motor windings (A)
   * @param torqueCurrent Current through motor windings (A)
   * @param temperature Temperature in °C
   * @param isConnected True if all status signals are valid and connected
   */
  public record MotorData(
      double position,
      double velocity,
      double acceleration,
      double appliedVolts,
      double supplyCurrent,
      double statorCurrent,
      double torqueCurrent,
      double temperature,
      boolean isConnected) {

    public MotorData() {
      this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
    }
  }
}
