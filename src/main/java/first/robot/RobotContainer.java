// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.commands.BigStateMachine;
import first.robot.commands.CommandFactory;
import first.robot.subsystems.endEffector.EE;
import first.robot.subsystems.endEffector.EEIOReal;
import first.robot.subsystems.endEffector.EEIOSim;
import first.robot.subsystems.launcher.Launcher;
import first.robot.subsystems.launcher.LauncherIOReal;
import first.robot.subsystems.launcher.LauncherIOSim;
import first.robot.subsystems.telescope.Telescope;
import first.robot.subsystems.telescope.TelescopeIOSim;
import first.robot.subsystems.vision.Vision;
import first.robot.util.PhoenixUtil;

public class RobotContainer {

  public final CommandNiDsXboxController driver;

  private final SendableChooser<Command> autoChooser;

  private final Telescope telescope;
  private final Launcher launcher;
  private final EE endEffector;

  // private final Vision vision;

  private final BigStateMachine statemachine;
  private final CommandFactory commandFactory;

  public RobotContainer() {
    driver = new CommandNiDsXboxController(0);

    autoChooser = new SendableChooser<>();

    telescope = new Telescope(new TelescopeIOSim());
    endEffector = new EE(new EEIOSim());
    launcher = new Launcher(new LauncherIOSim());

    // vision = new Vision();


    {// configuring state machine

      commandFactory = new CommandFactory(telescope, launcher, endEffector);

      statemachine = new BigStateMachine(
        commandFactory,
        driver.start(), // start
        driver.a(), // TODO: left bumper
        driver.leftTrigger(0.9), // left trigger
        driver.x(), // x
        driver.y(), // y
        driver.povRight(), // TODO: a
        driver.povUp(), // povUp
        driver.b(), // TODO: right bumper
        driver.rightTrigger(0.9) // right trigger
      );
    }

    setUpAutonomousCommand();
  }

  public void setUpAutonomousCommand() {
    // autoChooser.addOption("Auto Name", new PathPlannerAuto("Auto Nickname", bool inversion));

    SmartDashboard.putData("Commands/Autonomous", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command stateMachine() {
    return statemachine.SM();
  }

  public void periodic() {
    telescope.logIO();
    launcher.logIO();
    endEffector.logIO();
    statemachine.logData();
    PhoenixUtil.refreshAll();
  }

}
