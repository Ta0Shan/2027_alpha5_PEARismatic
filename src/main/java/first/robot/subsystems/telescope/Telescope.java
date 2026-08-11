// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.telescope;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;

public class Telescope extends Mechanism {
    private final TelescopeIO io;

    private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

    private TelescopeStates state = TelescopeStates.HOME;
    private double customSetpoint = 0.0;

    /** Creates a new Telescope. */
    public Telescope(TelescopeIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        SmartDashboard.putString("Mechanisms/Telescope/State", state.name());
        SmartDashboard.putNumber("Mechanisms/Telescope/Pivot/Setpoint Angle Deg", (state == TelescopeStates.LAUNCHER ? customSetpoint : state.getPivotAngleDeg()));
        SmartDashboard.putNumber("Mechanisms/Telescope/Pivot/Angle Deg", getPivotAngleDeg());
        SmartDashboard.putNumber("Mechanisms/Telescope/Pivot/Abs Encoder Angle Deg", Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition));
        SmartDashboard.putNumber("Mechanisms/Telescope/Arm/Extension Inches", getArmExtensionInches());
        SmartDashboard.putNumber("Mechanisms/Telescope/Arm/Setpoint Extension Inches", state.getArmExtensionInches());
        Logger.processInputs("Telescope", inputs);
        // TODO: see when logger starts working
    }

    public Command applyState(TelescopeStates state) {
        return run(co -> {
            this.state = state;
            if (state != TelescopeStates.LAUNCHER) { 
                // to prevent conflicts when we need to schedule a launcher cmd bc that's a specific case
                while(Math.abs((state.getPivotAngleDeg() - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition)) / state.getPivotAngleDeg()) > 0.05){
                        // && Math.abs((state.getArmExtensionInches() - getArmExtensionInches()) / state.getArmExtensionInches()) > 0.05) {
                    // functions as a timer, cmd gives up control when it's close to its setpoint
                    io.setPivotAngle(state.getPivotAngleDeg());
                    io.setArmExtension(state==TelescopeStates.CLUMB, state.getArmExtensionInches());
                    co.yield();
                }
            }
        }).named("APPLY STATE " + state.name());
    }

    public TelescopeStates getState() {
        return state;
    }

    public Command setPivotAngleDeg(double angleDeg) {
        // if (telescopeState != TelescopeStates.LAUNCHER) return Command.noRequirements(co -> {}).named("NOT LAUNCHING");
            return run(co -> {
                customSetpoint = angleDeg;
            while(Math.abs((customSetpoint - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition)) / customSetpoint) > 0.01) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                io.setPivotAngle(customSetpoint);
                co.yield();
            }
        }).named("PIVOT ANGLE " + angleDeg);
    }

    public double getPivotAngleDeg() {
        return Units.rotationsToDegrees(mean(
            inputs.pivot1Data.position(),
            -1 * inputs.pivot2Data.position(), // because this one follows in the opposed direction
            inputs.pivot3Data.position())
            / PivotConstants.REDUCTION);
    }

    public double getArmExtensionInches() {
        switch(state) {
            case CLUMB:
                return (mean(
                inputs.arm1Data.position(),
                inputs.arm2Data.position())
                / ArmConstants.EXTENSION_ROTOR_CIRCUMF_METERS // TODO: remember how you got this number
                / ArmConstants.CLIMB_REDUCTION);
            default:
                return (mean(
                    inputs.arm1Data.position(),
                    inputs.arm2Data.position())
                / ArmConstants.EXTENSION_ROTOR_CIRCUMF_METERS
                / ArmConstants.EXTENSION_REDUCTION);
        }
    }

    private double mean(double... values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return (sum / values.length);
    }
}
