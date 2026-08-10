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
import first.robot.subsystems.telescope.TelescopeConstants.EEConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.subsystems.telescope.TelescopeConstants.RollerStates;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;
import first.robot.subsystems.telescope.TelescopeConstants.WristStates;

public class Telescope extends Mechanism {

    private final TelescopeIO io;

    private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

    private TelescopeStates telescopeState = TelescopeStates.HOME;
    private double launchingSetpoint = 0.0;

    private WristStates wristState = WristStates.STOWED;
    private RollerStates rollerState = RollerStates.IDLE;

    /** Creates a new Telescope. */
    public Telescope(TelescopeIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        Logger.processInputs("Telescope", inputs);
        SmartDashboard.putString("Mechanisms/Telescope/State", telescopeState.name());
        SmartDashboard.putNumber("Mechanisms/Telescope/Pivot/Setpoint Angle Deg", (telescopeState == TelescopeStates.LAUNCHER ? launchingSetpoint : telescopeState.getPivotAngleDeg()));
        SmartDashboard.putNumber("Mechanisms/Telescope/Pivot/Angle Deg", getPivotAngleDeg());
        SmartDashboard.putNumber("Mechanisms/Telescope/Pivot/Abs Encoder Angle Deg", Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition));
        SmartDashboard.putNumber("Mechanisms/Telescope/Arm/Extension Inches", getArmExtensionInches());
        SmartDashboard.putNumber("Mechanisms/Telescope/Arm/Setpoint Extension Inches", getArmExtensionInches());
        // TODO: see when logger starts working
    }

    public Command applyState(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState) {
        return run(co -> {
            this.telescopeState = telescopeState;
            this.wristState = wristState;
            this.rollerState = rollerState;
            while(Math.abs((telescopeState.getPivotAngleDeg() - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition)) / telescopeState.getPivotAngleDeg()) > 0.05
                    && Math.abs((telescopeState.getArmExtensionInches() - getArmExtensionInches()) / telescopeState.getArmExtensionInches()) > 0.05
                    && Math.abs((wristState.getAngleDeg() - (Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION))) / wristState.getAngleDeg() > 0.05) {
                io.setPivotAngle(telescopeState.getPivotAngleDeg());
                io.setArmExtension(telescopeState == TelescopeStates.CLUMB, telescopeState.getArmExtensionInches());
                io.setWristAngle(wristState.getAngleDeg());
                io.setRollerVolts(rollerState.getVoltage());
                co.yield();
            }
        }).named(String.format("TELE %s | EE %s %s", telescopeState.name(), wristState.name(), rollerState.name()));
    }

    public Command applyState(TelescopeStates state) {
        return run(co -> {
            telescopeState = state;
            if (telescopeState != TelescopeStates.LAUNCHER) { 
                // to prevent conflicts when we need to schedule a launcher cmd bc that's a specific case
                while((state.getPivotAngleDeg() - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition)) / state.getPivotAngleDeg() > 0.05 
                        && (state.getArmExtensionInches() - getArmExtensionInches()) / state.getArmExtensionInches() > 0.05) {
                    // functions as a timer, cmd gives up control when it's close to its setpoint
                    io.setPivotAngle(state.getPivotAngleDeg());
                    io.setArmExtension(state==TelescopeStates.CLUMB, state.getArmExtensionInches());
                    co.yield();
                }
            }
        }).named("APPLY STATE " + state.name());
    }

    public TelescopeStates getState() {
        return telescopeState;
    }

    public Command setPivotAngleDeg(double angleDeg) {
        if (telescopeState != TelescopeStates.LAUNCHER) return Command.noRequirements(co -> {}).named("NOT LAUNCHING");
            return run(co -> {
            launchingSetpoint = angleDeg;
            // TODO: work sim
            while((launchingSetpoint - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition)) / launchingSetpoint > 0.05) {
                // functions as a timer, cmd gives up control when it's close to its setpoint
                io.setPivotAngle(launchingSetpoint);
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
        switch(telescopeState) {
            case CLUMB:
                return (mean(
                inputs.arm1Data.position(),
                inputs.arm2Data.position())
                / ArmConstants.EXTENSION_ROTOR_CIRCUMF_INCHES // TODO: remember how you got this number
                / ArmConstants.CLIMB_REDUCTION);
            default:
                return (mean(
                    inputs.arm1Data.position(),
                    inputs.arm2Data.position())
                / ArmConstants.EXTENSION_ROTOR_CIRCUMF_INCHES
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
