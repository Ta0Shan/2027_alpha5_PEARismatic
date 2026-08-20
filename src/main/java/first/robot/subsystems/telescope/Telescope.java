// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.telescope;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;

import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;

public class Telescope extends Mechanism {
    private final TelescopeIO io;

    private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

    @AutoLogOutput(key="Mechanisms/Telescope/State") private TelescopeStates state = TelescopeStates.HOME;

    /** Creates a new Telescope. */
    public Telescope(TelescopeIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        Logger.processInputs("Telescope", inputs);
        Logger.recordOutput("Mechanisms/Telescope/Pivot/Setpoint Angle Deg", state.pivotAngleDeg);
        Logger.recordOutput("Mechanisms/Telescope/Pivot/Angle Deg", getPivotAngleDeg());
        Logger.recordOutput("Mechanisms/Telescope/Pivot/Abs Encoder Angle Deg", Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition));

        Logger.recordOutput("Mechanisms/Telescope/Arm/Extension Inches", getArmExtensionInches());
        Logger.recordOutput("Mechanisms/Telescope/Arm/Setpoint Extension Inches", getArmSetpoint(state));
        Logger.recordOutput("Mechanisms/Telescope/Arm/Dog Shifter State", (inputs.armServoAppliedPulseWidth == ArmConstants.CLIMB_PULSE_WIDTH_uS ? "climb" : "extension"));
    }

    public Command applyState(TelescopeStates state) {
        return run(co -> {
            Debouncer setpointDebouncer = new Debouncer(0.2);
            this.state = state;
            while(setpointDebouncer.calculate(
                    Math.abs((state.pivotAngleDeg - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition))) > 0.5
                    || Math.abs(getArmSetpoint(state) - getArmExtensionInches()) > 0.05)
                ) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                io.setPivotAngleDeg(state.pivotAngleDeg);
                io.setArmExtensionIn(state==TelescopeStates.CLUMB, state.armExtensionInches);
                co.yield();
            }
            io.shiftDogs(state == TelescopeStates.CLIMB_RAISED);
        }).named("TELE " + state.name());
    }

    public TelescopeStates getState() {
        return state;
    }

    public double getPivotAngleDeg() {
        return Units.rotationsToDegrees(mean(
            inputs.pivot1Data.position(),
            -inputs.pivot2Data.position(), // because this one follows in the opposed direction
            inputs.pivot3Data.position())
            / PivotConstants.REDUCTION
        );
    }

    public double getArmExtensionInches() {
        return mean(
            inputs.arm1Data.position(),
            inputs.arm2Data.position())
            / (state == TelescopeStates.CLUMB ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION)
            // / (inputs.armServoAppliedPulseWidth == ArmConstants.CLIMB_PULSE_WIDTH_uS ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION)
            // / ArmConstants.EXTENSION_REDUCTION
            * Units.metersToInches(ArmConstants.ROTOR_CIRCUMF_METERS)
            // + (state == TelescopeStates.CLUMB ? (
            //     Units.inchesToMeters(TelescopeStates.CLIMB_RAISED.getArmExtensionInches())
            //         / ArmConstants.ROTOR_CIRCUMF_METERS
            //         * ArmConstants.EXTENSION_REDUCTION)
            //     : 0)
        ;
    }

    private double getArmSetpoint(TelescopeStates state) {
        // return (state==TelescopeStates.CLUMB ? 2 : state.getArmExtensionInches());
        return state.armExtensionInches;
    }

    private double mean(double... values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return (sum / values.length);
    }
}
