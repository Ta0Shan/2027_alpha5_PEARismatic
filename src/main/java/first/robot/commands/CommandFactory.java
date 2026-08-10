// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.units.measure.Time;

import first.robot.Constants.SuperstructureStates;
import first.robot.subsystems.endEffector.EE;
import first.robot.subsystems.endEffector.EEConstants.RollerStates;
import first.robot.subsystems.launcher.Launcher;
import first.robot.subsystems.telescope.Telescope;

/** Add your docs here. */
public class CommandFactory {

    private final Telescope telescope;
    private final EE endEffector;
    private final Launcher launcher;

    private SuperstructureStates superstructureState = SuperstructureStates.HOME;

    // private final Drive drive;

    public CommandFactory(Telescope telescope, Launcher launcher, EE endEffector) {
        this.telescope = telescope;
        this.launcher = launcher;
        this.endEffector = endEffector;
    }

    public Command pause(double seconds) {
        return Command.noRequirements(co -> {co.wait(Time.ofBaseUnits(seconds, Seconds));}).named("WAIT");
    }
    
    private Command instantApplyState(SuperstructureStates state) {
        return Command.parallel(
            Command.noRequirements(co -> {superstructureState = state;}).named("SET STATE " + superstructureState.name()),
            telescope.applyState(state.getTelescopeState()),
            endEffector.applyState(state.getWristState(), state.getRollerState()),
            launcher.setLauncherRPS(state.getLauncherRPS())
        ).named(String.format("%s :: TELE %s | EE %s %s | LAUNCHER %f", state.name(), state.getTelescopeState().name(), state.getWristState().name(), state.getRollerState().name(), state.getLauncherRPS()));
    }

    public Command applyState(SuperstructureStates state) {
        return Command.parallel(
            instantApplyState(state),
            hold()
        ).named(instantApplyState(state).name());
    }
    
    public Command classify() {
        return Command.requiring(telescope, launcher, endEffector).executing(co -> {
            co.fork(instantApplyState(SuperstructureStates.LAUNCHER));
            co.awaitAll(
                telescope.setPivotAngleDeg(20),
                launcher.setLauncherRPS(60) // TODO: get a calc
            );
            co.await(pause(1));
        }).named("CLASSIFY");
    }
    public Command shuttle() {// TODO: right now its the same thing but you gotta figure out how to get vision/pos working to change it
                                // and also maybe CSV reading for fun
        return Command.requiring(telescope, launcher, endEffector).executing(co -> {
            co.fork(instantApplyState(SuperstructureStates.LAUNCHER));
            co.awaitAll(
                telescope.setPivotAngleDeg(20),
                launcher.setLauncherRPS(100)
            );
            co.await(pause(1));
        }).named("SHUTTLE");
    }

    public Command coloredAlign() {
        return pause(1);
    }

    public Command neutralAlign() {
        return pause(1);
    }
    // TODO: implement colored alignment rather than placement, use a switchTo().whenComplete() to go from level -> align 

    public Command score() { // TODO: put alignment in here
        return Command.requiring(endEffector).executing(co -> {
            co.fork(endEffector.applyState(RollerStates.REV));
            co.await(pause(0.5));
            // if you're using this in real life you would use the color sensor here with like endEffector.waitUntilScored();
        }).named("SCORE");
    }

    public Command climb() {
        return Command.requiring(telescope).executing(co -> {
            co.await(applyState(SuperstructureStates.CLUMB));
        }).named("CLIMB");
    }

    public Command hold() {
        return Command.noRequirements(co -> {
            while(true) {
                co.yield();
            }
        }).named("HOLD");
    }

    public SuperstructureStates getSuperstructureState() {
        return superstructureState;
    }

}
