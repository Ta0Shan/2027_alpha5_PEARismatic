// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.smartdashboard.SmartDashboard;

import org.wpilib.command3.Trigger;

import first.robot.Constants.SuperstructureStates;

/** Add your docs here. */
public class BigStateMachine {

    private final CommandFactory commands;

    // Button triggers
    private final Trigger homeTrigger;
    private final Trigger intakeTrigger;
    private final Trigger outtakeTrigger;
    private final Trigger goToL1Trigger;
    private final Trigger goToL2Trigger;
    private final Trigger goToClimbTrigger;

    private final Trigger primaryScoreTrigger;
    private final Trigger secondaryScoreTrigger;

    // Robot State Triggers
    private Trigger isFront;

    public BigStateMachine(CommandFactory commandFactory,
                            Trigger home, 
                            Trigger intake,
                            Trigger outtake,
                            Trigger goToL1, 
                            Trigger goToL2, 
                            Trigger goToClimb, 
                            Trigger primaryScore,
                            Trigger secondaryScore
                            ) {

        commands = commandFactory;

        homeTrigger = home;
        intakeTrigger = intake;
        outtakeTrigger = outtake;
        goToL1Trigger = goToL1;
        goToL2Trigger = goToL2;
        goToClimbTrigger = goToClimb;
        primaryScoreTrigger = primaryScore;
        secondaryScoreTrigger = secondaryScore;

        isFront = new Trigger(() -> true); // TODO: pos
    }

    public StateMachine SM() {
        // Defining States
            StateMachine stateMachine = new StateMachine("STATE MACHINE");

            // mechanism states / starters
            State HOME = stateMachine.addState(commands.applyState(SuperstructureStates.HOME)); // resting state
            State INTAKING = stateMachine.addState(commands.applyState(SuperstructureStates.INTAKING)); // arm down, intaking
            State OUTTAKING = stateMachine.addState(commands.applyState(SuperstructureStates.OUTTAKING)); // arm down, outtaking
            State L1_FRONT = stateMachine.addState(commands.applyState(SuperstructureStates.L1_FRONT)); // arm up forwards, prepped for L1
            State L1_BACK = stateMachine.addState(commands.applyState(SuperstructureStates.L1_BACK)); // arm up backwards, prepped for L1
            State L2_FRONT = stateMachine.addState(commands.applyState(SuperstructureStates.L2_FRONT)); // arm up forwards, prepped for L2
            State L2_BACK = stateMachine.addState(commands.applyState(SuperstructureStates.L2_BACK)); // arm up backwards, prepped for L2
            State CLIMB_RAISED = stateMachine.addState(commands.applyState(SuperstructureStates.CLIMB_RAISED)); // arm up 90, prepped for climb
            State IDLING = stateMachine.addState(commands.hold());

            // alignment states / in-betweens
            State CLASSIFY = stateMachine.addState(commands.classify()); // TODO: pose when akit and drive works
            State SHUTTLE = stateMachine.addState(commands.shuttle()); // TODO: ditto ^^^
            State COLORED = stateMachine.addState(commands.coloredAlign()); // TODO: ditto ^^^
            State NEUTRAL = stateMachine.addState(commands.neutralAlign()); // TODO: ditto ^^^

            // scoring states / finals
            State SCORE = stateMachine.addState(commands.score());
            State CLIMB = stateMachine.addState(commands.climb());

        // Binding Triggers
            // we will always go to HOME when homeTrigger is triggered
            stateMachine.setInitialState(HOME);
            stateMachine.switchFromAny().to(HOME).when(homeTrigger);

            // HOME, INTAKE, and OUTTAKE can freely switch to each other
            stateMachine.switchFromAny(HOME, OUTTAKING, IDLING).to(INTAKING).when(intakeTrigger);
            stateMachine.switchFromAny(HOME, INTAKING, IDLING).to(OUTTAKING).when(outtakeTrigger);
            INTAKING.switchTo(HOME).when(intakeTrigger.negate());
            OUTTAKING.switchTo(HOME).when(outtakeTrigger.negate());
            // you cannot go from any scoring level to INTAKE/OUTTAKE without passing back through HOME



            // based on FRONT or BACK, HOME will go to L1/L2 on goToL1Trigger/goToL2Trigger triggers
            // L1 and L2 can also swap on if driver misclicks
            stateMachine.switchFromAny(HOME, L2_FRONT, L2_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? L1_FRONT : L1_BACK).when(goToL1Trigger);
            stateMachine.switchFromAny(HOME, L1_FRONT, L1_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? L2_FRONT : L2_BACK).when(goToL2Trigger);

            // no matter the direction or scoring state, pressing scoreTrigger while align and place
            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, IDLING)
                        .to(COLORED).when(primaryScoreTrigger.risingEdge());
            // using risingEdge() trigger to make sure it's pressed, not held from a previous state

            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, IDLING)
                        .to(NEUTRAL).when(secondaryScoreTrigger.risingEdge());
            
            // if the driver second guesses while aligning, letting go of the score trigger will enter a IDLING state
            // IDLING maintains current setpoints so driver can continue or pivot if they want to
            COLORED.switchTo(IDLING).when(primaryScoreTrigger.negate());
            NEUTRAL.switchTo(IDLING).when(secondaryScoreTrigger.negate());
            // ideally the driver shouldn't second guess but it's always nice to have yk
            
            // once the robot is aligned, switch from alignment state to scoring state
            stateMachine.switchFromAny(COLORED, NEUTRAL).to(SCORE).whenComplete();

            // LAUNCH is the normal scoring mech for the launcher
            HOME.switchTo(CLASSIFY).when(primaryScoreTrigger.risingEdge());
            // SHUTTLE is the alternate scoring mech for the launcher
            HOME.switchTo(SHUTTLE).when(secondaryScoreTrigger.risingEdge());

            // if the driver second guesses while about to shoot, letting go of the score trigger will go back to HOME
            CLASSIFY.switchTo(HOME).when(primaryScoreTrigger.negate());
            SHUTTLE.switchTo(HOME).when(secondaryScoreTrigger.negate());
            
            // once the robot is aligned, switch from alignment state to scoring state
            stateMachine.switchFromAny(CLASSIFY, SHUTTLE).to(SCORE).whenComplete();

            // any scoring state will always return to HOME after it's done
            stateMachine.switchFromAny(SCORE).to(HOME).whenComplete();



            // when goToClimbTrigger is triggered, we will begin climb sequence with CLIMB_RAISED
            HOME.switchTo(CLIMB_RAISED).when(goToClimbTrigger);
            // since CLIMB engages the 1-time dog shifter, we want to make sure it's deliberate by requiring two buttons
            CLIMB_RAISED.switchTo(CLIMB).when(primaryScoreTrigger.and(secondaryScoreTrigger));
            // marks CLIMB as the final command
            CLIMB.exitStateMachine().whenComplete();

            return stateMachine;
    }

    public void logState() {
        SmartDashboard.putString("Mechanisms/Superstructure State", commands.getSuperstructureState().name());
    }

}
