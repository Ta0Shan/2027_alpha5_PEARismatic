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

    private final CommandFactory factory;

    // Button triggers
    private final Trigger homeTrigger;
    private final Trigger intakeTrigger;
    private final Trigger outtakeTrigger;
    private final Trigger L1Trigger;
    private final Trigger L2Trigger;
    private final Trigger classifierTrigger;
    private final Trigger climbTrigger;

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
                            Trigger classifier, 
                            Trigger primaryScore,
                            Trigger secondaryScore
                            ) {

        factory = commandFactory;

        homeTrigger = home;
        intakeTrigger = intake;
        outtakeTrigger = outtake;
        L1Trigger = goToL1;
        L2Trigger = goToL2;
        classifierTrigger = classifier;
        climbTrigger = goToClimb;
        primaryScoreTrigger = primaryScore;
        secondaryScoreTrigger = secondaryScore;

        isFront = new Trigger(() -> true); // TODO: pos
    }

    public StateMachine SM() {
        // Defining States
            StateMachine stateMachine = new StateMachine("STATE MACHINE");

            // mechanism states / starters
            State HOME = stateMachine.addState(factory.applyState(SuperstructureStates.HOME)); // resting state
            State INTAKING = stateMachine.addState(factory.applyState(SuperstructureStates.INTAKING)); // arm down, intaking
            State OUTTAKING = stateMachine.addState(factory.applyState(SuperstructureStates.OUTTAKING)); // arm down, outtaking
            State L1_FRONT = stateMachine.addState(factory.applyState(SuperstructureStates.L1_FRONT)); // arm up forwards, prepped for L1
            State L1_BACK = stateMachine.addState(factory.applyState(SuperstructureStates.L1_BACK)); // arm up backwards, prepped for L1
            State L2_FRONT = stateMachine.addState(factory.applyState(SuperstructureStates.L2_FRONT)); // arm up forwards, prepped for L2
            State L2_BACK = stateMachine.addState(factory.applyState(SuperstructureStates.L2_BACK)); // arm up backwards, prepped for L2
            State CLASSIFIER_FRONT = stateMachine.addState(factory.applyState(SuperstructureStates.CLASSIFIER_FRONT)); // arm up forwards, prepped for lower classifier
            State CLASSIFIER_BACK = stateMachine.addState(factory.applyState(SuperstructureStates.CLASSIFIER_BACK)); // arm up backwards, prepped for upper classifier
            State CLIMB_RAISED = stateMachine.addState(factory.applyState(SuperstructureStates.CLIMB_RAISED)); // arm up 90, prepped for climb
            State IDLING = stateMachine.addState(factory.hold());

            // alignment states / in-betweens
            State SHUTTLE = stateMachine.addState(factory.shuttle()); // TODO: pose when akit and drive works
            State COLORED = stateMachine.addState(factory.coloredAlign()); // TODO: ditto ^^^
            State NEUTRAL = stateMachine.addState(factory.neutralAlign()); // TODO: ditto ^^^

            // scoring states / finals
            State SCORE = stateMachine.addState(factory.score());
            State CLIMB = stateMachine.addState(factory.climb());

        // Binding Triggers
            // we will always go to HOME when homeTrigger is triggered
            stateMachine.setInitialState(HOME);
            stateMachine.switchFromAny().to(HOME).when(homeTrigger);

            // HOME, INTAKE, and OUTTAKE can freely switch to each other
            stateMachine.switchFromAny(HOME, OUTTAKING).to(INTAKING).when(intakeTrigger);
            stateMachine.switchFromAny(HOME, INTAKING, IDLING).to(OUTTAKING).when(outtakeTrigger);
            INTAKING.switchTo(HOME).when(intakeTrigger.negate());
            OUTTAKING.switchTo(HOME).when(outtakeTrigger.negate());
            // you cannot go from any scoring level to INTAKE/OUTTAKE without passing back through HOME



            // based on FRONT or BACK, HOME will go to L1/L2/Classifier on goToL1Trigger/goToL2Trigger triggers
            // L1 and L2 can also swap on if driver misclicks
            stateMachine.switchFromAny(HOME, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? L1_FRONT : L1_BACK).when(L1Trigger);
            stateMachine.switchFromAny(HOME, L1_FRONT, L1_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? L2_FRONT : L2_BACK).when(L2Trigger);
            stateMachine.switchFromAny(HOME, L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? CLASSIFIER_FRONT : CLASSIFIER_BACK).when(classifierTrigger);

            // no matter the direction or scoring state, pressing scoreTrigger will align
            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                        .to(COLORED).when(primaryScoreTrigger.risingEdge());
            // using risingEdge() trigger to make sure it's pressed, not held from a previous state

            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                        .to(NEUTRAL).when(secondaryScoreTrigger.risingEdge());
            // both primary and secondary score triggers can CLASSIFY - case will be handled in command factory
            
            // if the driver second guesses while aligning, letting go of the score trigger will enter a IDLING state
            // IDLING maintains current setpoints so driver can continue or pivot if they want to
            COLORED.switchTo(IDLING).when(primaryScoreTrigger.negate());
            NEUTRAL.switchTo(IDLING).when(secondaryScoreTrigger.negate());
            // ideally the driver shouldn't second guess but it's always nice to have yk
            
            // once the robot is aligned, switch from alignment state to scoring state
            stateMachine.switchFromAny(COLORED, NEUTRAL).to(SCORE).whenComplete();

            // SHUTTLE is the "scoring" mech for the launcher
            HOME.switchTo(SHUTTLE).when(primaryScoreTrigger.risingEdge());

            // if the driver second guesses while about to shoot, letting go of the score trigger will go back to HOME
            SHUTTLE.switchTo(HOME).when(primaryScoreTrigger.negate());
            
            // once the robot is aligned, switch from alignment state to scoring state
            SHUTTLE.switchTo(SCORE).whenComplete();

            // scoring state will always return to HOME after it's done
            SCORE.switchTo(HOME).whenComplete();



            // when goToClimbTrigger is triggered, we will begin climb sequence with CLIMB_RAISED
            HOME.switchTo(CLIMB_RAISED).when(climbTrigger);
            // since CLIMB engages the 1-time dog shifter, we want to make sure it's deliberate by requiring two buttons
            CLIMB_RAISED.switchTo(CLIMB).when(primaryScoreTrigger.and(secondaryScoreTrigger));
            // marks CLIMB as the final command
            CLIMB.exitStateMachine().whenComplete();

            return stateMachine;
    }

    public void logData() {
        SmartDashboard.putString("Mechanisms/Superstructure State", factory.getSuperstructureState().name());
        SmartDashboard.putNumber("Mechanisms/End Effector/Wrist/Angle From Floor Deg", factory.getEEAngleFromFloorDeg());
    }

}
