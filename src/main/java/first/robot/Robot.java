// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent.CompletedWithError;
import org.wpilib.command3.SchedulerEvent.Interrupted;
import org.wpilib.command3.SchedulerEvent.Canceled;
import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final Scheduler scheduler;

  private final List<String> problemCommands;

  public Robot() {
    robotContainer = new RobotContainer();

    scheduler = Scheduler.getDefault();
    scheduler.addPeriodic(() -> robotContainer.periodic());

    problemCommands = new ArrayList<String>();
  }

  @Override
  public void robotPeriodic() {
    scheduler.run();

    Command[] runningCommands = scheduler.getRunningCommands().toArray(Command[]::new);
    String[] names = new String[runningCommands.length];
    for (int i = 0; i < runningCommands.length; i++) {
      names[i] = runningCommands[i].name();
    }
    
    SmartDashboard.putStringArray("Commands/Running", names);

    // outputs important events: non-idle interruptions and errored completions
    scheduler.addEventListener(event -> {
      String message;
      switch(event) {
        case CompletedWithError(Command cmd, Error error, long time):
          message = ((double)Math.round(time / 1000.0) / 1000.0) + " | Error with " + cmd.name() + ": " + error.toString();
          if (!problemCommands.contains(message)) {
            problemCommands.add(0, message);
          }
          break;
        case Interrupted(Command cmd, Command inter, long time):
          message = ((double)Math.round(time / 1000.0) / 1000.0) + " | " + cmd.name() + " interrupted by " + inter.name();
          if(!cmd.name().contains("[IDLE]") && !problemCommands.contains(message)) {
            problemCommands.add(0, message);
          }
          break;
          // cancellations not rly that important lol
          // case Canceled(Command cmd, long time):
          //   message = ((double)Math.round(time / 1000.0) / 1000.0) + " | " + cmd.name() + " canceled";
          //   if(!cmd.name().contains("[IDLE]") && !problemCommands.contains(message)) {
          //     problemCommands.add(0, message);
          //   }
          // break;
        default:
          break;
      }
    });

    SmartDashboard.putStringArray("Commands/Special Events/List", problemCommands.toArray(String[]::new));
    SmartDashboard.putString("Commands/Special Events/Recent ", (problemCommands.size() > 0 ? problemCommands.get(0) : ""));
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      scheduler.schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      scheduler.cancel(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    scheduler.schedule(robotContainer.stateMachine());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    scheduler.cancelAll();
  }

  @Override
  public void utilityInit() {}

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}
}
