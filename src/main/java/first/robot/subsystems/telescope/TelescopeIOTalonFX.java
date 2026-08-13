// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.telescope;

import java.util.function.DoubleSupplier;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import first.robot.Constants;
import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.util.PearadoxTalonFX;
import first.robot.util.PhoenixUtil;
import first.robot.util.EnergyTracker.Subsystem;

/** Add your docs here. */
public abstract class TelescopeIOTalonFX implements TelescopeIO {
    protected final PearadoxTalonFX pivot1;
    protected final PearadoxTalonFX pivot2;
    protected final PearadoxTalonFX pivot3;

    protected final CANcoder absoluteEncoder;

    protected final PearadoxTalonFX arm1;
    protected final PearadoxTalonFX arm2;

    protected final MotionMagicDutyCycle mmDutyCycle;
    protected final Follower follower;

    public TelescopeIOTalonFX() {
        pivot1 = new PearadoxTalonFX(PivotConstants.PIVOT_1_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            PivotConstants.PIVOT_CONFIG(),
            Subsystem.TELESCOPE_PIVOT);
            
        pivot2 = new PearadoxTalonFX(PivotConstants.PIVOT_2_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            PivotConstants.PIVOT_CONFIG(),
            Subsystem.TELESCOPE_PIVOT);

        pivot3 = new PearadoxTalonFX(PivotConstants.PIVOT_3_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            PivotConstants.PIVOT_CONFIG(),
            Subsystem.TELESCOPE_PIVOT);

        absoluteEncoder = new CANcoder(PivotConstants.CANCODER_ID, Constants.SUPERSTRUCTURE_CAN_BUS);
        PhoenixUtil.tryUntilOk(5, () -> absoluteEncoder.getConfigurator().apply(PivotConstants.CANCODER_CONFIG()));

        arm1 = new PearadoxTalonFX(ArmConstants.ARM_1_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            ArmConstants.CONFIG(),
            Subsystem.TELESCOPE_EXTENSION);

        arm2 = new PearadoxTalonFX(ArmConstants.ARM_2_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            ArmConstants.CONFIG(),
            Subsystem.TELESCOPE_EXTENSION);

        
        // setting up control modes
        mmDutyCycle = new MotionMagicDutyCycle(0);
        follower = new Follower(0, null);

        pivot2.setControl(follower.withLeaderID(PivotConstants.PIVOT_1_ID).withMotorAlignment(MotorAlignmentValue.Opposed));
        pivot3.setControl(follower.withMotorAlignment(MotorAlignmentValue.Aligned));

        arm2.setControl(follower.withLeaderID(ArmConstants.ARM_1_ID).withMotorAlignment(MotorAlignmentValue.Aligned));

        syncMotorsWithEncoder();
    }

    protected void syncMotorsWithEncoder() {
        PhoenixUtil.tryUntilOk(5, () -> pivot1.setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * PivotConstants.REDUCTION));
        PhoenixUtil.tryUntilOk(5, () -> pivot2.setPosition(-absoluteEncoder.getAbsolutePosition().getValueAsDouble() * PivotConstants.REDUCTION));
        PhoenixUtil.tryUntilOk(5, () -> pivot3.setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * PivotConstants.REDUCTION));
    }

    public void updateInputs(TelescopeIOInputs inputs) {
        inputs.pivot1Data = pivot1.getData();
        inputs.pivot2Data = pivot2.getData();
        inputs.pivot3Data = pivot3.getData();
        
        inputs.pivotAbsEncoderPosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

        inputs.arm1Data = arm1.getData();
        inputs.arm2Data = arm2.getData();
    }

    public void setPivotAngleDeg(double angleDeg) {
        double motorSetpoint = Units.degreesToRotations(angleDeg) * PivotConstants.REDUCTION;
        pivot1.setControl(mmDutyCycle.withPosition(motorSetpoint));
    }

    public void setPivotAngleDeg(double angleDeg, DoubleSupplier ff) {
        double motorSetpoint = Units.degreesToRotations(angleDeg) * PivotConstants.REDUCTION;
        pivot1.setControl(mmDutyCycle.withPosition(motorSetpoint).withFeedForward(ff.getAsDouble()));
    }

    public void setArmExtensionIn(boolean isClimbing, double extensionInches) {
        double motorSetpoint = (Units.inchesToMeters(extensionInches) / ArmConstants.ROTOR_CIRCUMF_METERS)
                * (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION);
        arm1.setControl(mmDutyCycle.withPosition(motorSetpoint));
    }

    public void setArmExtensionIn(boolean isClimbing, double extensionInches, DoubleSupplier ff) {
        double motorSetpoint = (Units.inchesToMeters(extensionInches) / ArmConstants.ROTOR_CIRCUMF_METERS)
                 * (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION);
            arm1.setControl(mmDutyCycle.withPosition(motorSetpoint).withFeedForward(ff.getAsDouble()));
    }

}
