package first.robot.subsystems.telescope;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import first.robot.Constants;
import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.util.VariableLengthArmSim;

public class TelescopeIOSim extends TelescopeIOTalonFX {
    private final TalonFXSimState pivot1SimState;
    private final TalonFXSimState pivot2SimState;
    private final TalonFXSimState pivot3SimState;
    private final CANcoderSimState absoluteEncoderSimState;
    private final TalonFXSimState arm1SimState;

    private final SingleJointedArmSim pivotPhysicsSim = new SingleJointedArmSim(
        DCMotor.getKrakenX44Foc(3),
        PivotConstants.REDUCTION,
        SingleJointedArmSim.estimateMOI(ArmConstants.MIN_LENGTH_METERS, ArmConstants.MASS_KG),
        ArmConstants.MIN_LENGTH_METERS,
        Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        false,
        Units.degreesToRadians(PivotConstants.STARTING_ANGLE_DEG)
    );

    // private final VariableLengthArmSim psim = new VariableLengthArmSim(
    //     DCMotor.getKrakenX60Foc(3), 
    //     PivotConstants.REDUCTION, 0, 0, 0, 0, 0, false
    // );

    public TelescopeIOSim() {
        super();
        pivot1SimState = pivot1.getSimState();
        pivot2SimState = pivot2.getSimState();
        pivot3SimState = pivot3.getSimState();
        absoluteEncoderSimState = absoluteEncoder.getSimState();
        arm1SimState = arm1.getSimState();
    }

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        super.updateInputs(inputs);

        pivot1SimState.setSupplyVoltage(12);
        pivot2SimState.setSupplyVoltage(12);
        pivot3SimState.setSupplyVoltage(12);
        pivotPhysicsSim.setInputVoltage((pivot1SimState.getMotorVoltage() + pivot2SimState.getMotorVoltage() + pivot3SimState.getMotorVoltage()) / 3);

        pivotPhysicsSim.update(Constants.UPDATE_FREQ_SEC);

        pivot1SimState.setRawRotorPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()) * PivotConstants.REDUCTION);
        pivot2SimState.setRawRotorPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()) * PivotConstants.REDUCTION);
        pivot3SimState.setRawRotorPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()) * PivotConstants.REDUCTION);
        pivot1SimState.setRotorVelocity(Units.radiansToRotations(pivotPhysicsSim.getVelocity()) * PivotConstants.REDUCTION);
        pivot2SimState.setRotorVelocity(Units.radiansToRotations(pivotPhysicsSim.getVelocity()) * PivotConstants.REDUCTION);
        pivot3SimState.setRotorVelocity(Units.radiansToRotations(pivotPhysicsSim.getVelocity()) * PivotConstants.REDUCTION);
        absoluteEncoderSimState.setRawPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()));
    }
}
