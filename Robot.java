// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.CANDriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;


//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private boolean inRange = false;
  private final Timer mTimer = new Timer();
 
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    
    //LimelightHelpers.setupPortForwardingUSB(0);
    //LimelightHelpers.setupPortForwardingUSB(1);
   

    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
   
    double ty = LimelightHelpers.getTY("limelight-alpha");
    //System.out.println("Limelight.TY: " + ty);

    
  }
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);;
    }
     if (currentAutoCommand != null) {
    currentAutoCommand.cancel();
    currentAutoCommand = null;
  }
  if (currentBallCommand != null) {
    currentBallCommand.cancel();
    currentBallCommand = null;
  }
  
  // Reset state variables
  inRange = false;
  currentState = "";
  
  // Reset timer
  mTimer.stop();
  mTimer.reset();
  
  // Optional: cancel all scheduled commands to be extra safe
  CommandScheduler.getInstance().cancelAll();
  }


private Command currentAutoCommand = null;
private Command currentBallCommand = null;
private String currentState = "";

public void autonomousPeriodic() {
  double tY = LimelightHelpers.getTY("limelight-alpha");
  double tX = LimelightHelpers.getTX("limelight-alpha");
  double kp = -.007;
  boolean tV = LimelightHelpers.getTV("limelight-alpha");

 
if (tV) {
  // Check if we've entered the range
  if (tY > 18 && tY < 22 && !inRange) {
    // JUST entered the range - immediately transition to aligned
    inRange = true;
    
    // Cancel any driving commands immediately
    if (currentAutoCommand != null) {
      currentAutoCommand.cancel();
    }
    if (currentBallCommand != null) {
      currentBallCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();
    
    currentAutoCommand = m_robotContainer.driveSubsystem
        .driveArcade(() -> 0, () -> tX*kp);
    currentAutoCommand.schedule();
    
    mTimer.reset();
    mTimer.start();
    currentState = "aligned";
    //System.out.println("ENTERED RANGE - stopping and starting timer");
  }
  
  if (!inRange) {
    // Not in range yet - keep driving to get there
    if (tY < 18) {
      if (!currentState.equals("backward")) {
        if (currentAutoCommand != null) currentAutoCommand.cancel();
        if (currentBallCommand != null) {
          currentBallCommand.cancel();
          currentBallCommand = null;
        }
        m_robotContainer.ballSubsystem.runOnce(() -> m_robotContainer.ballSubsystem.stop()).schedule();
        if (tX < 5 && tX > -5) {
        currentAutoCommand = m_robotContainer.driveSubsystem.driveArcade(() -> -0.4, () -> 0);
        }
        else {
          currentAutoCommand = m_robotContainer.driveSubsystem.driveArcade(() -> -0.4, () -> tX*kp);
        }
        currentAutoCommand.schedule();
        currentState = "backward";
        //System.out.println("back - tY: " + tY);
      }
    } else if (tY > 22) {
      if (!currentState.equals("forward")) {
        if (currentAutoCommand != null) currentAutoCommand.cancel();
        if (currentBallCommand != null) {
          currentBallCommand.cancel();
          currentBallCommand = null;
        }
        m_robotContainer.ballSubsystem.runOnce(() -> m_robotContainer.ballSubsystem.stop()).schedule();
        if (tX < 5 && tX > -5) {
        currentAutoCommand = m_robotContainer.driveSubsystem.driveArcade(() -> 0.4, () -> 0);
        }
        else {
          currentAutoCommand = m_robotContainer.driveSubsystem.driveArcade(() -> 0.4, () -> tX*kp);
        }
        currentAutoCommand.schedule();
        currentState = "forward";
        //System.out.println("forward - tY: " + tY);
      }
    }
  } else {
    // We're in range - handle shooting sequence
    //System.out.println("In range - timer at: " + mTimer.get());
    
    // if (mTimer.get() < 1.0) {
    //   if (currentBallCommand == null || currentBallCommand.isFinished()) {
    //     currentBallCommand = m_robotContainer.ballSubsystem.spinUpCommand();
    //     currentBallCommand.schedule();
    //     System.out.println("Spinning up");
    //   }
    // } else {
      if (currentBallCommand == null || currentBallCommand.isFinished()) {
        currentBallCommand = m_robotContainer.ballSubsystem.launchCommand();
        currentBallCommand.schedule();
        //System.out.println("LAUNCHING");
      }
    //}
  }
  
} else {
  // No target visible - reset everything
  inRange = false;
  currentState = "";
  if (currentAutoCommand != null) {
    currentAutoCommand.cancel();
    currentAutoCommand = null;
  }
  if (currentBallCommand != null) {
    currentBallCommand.cancel();
    currentBallCommand = null;
  }
  mTimer.stop();
  mTimer.reset();
  m_robotContainer.driveSubsystem.driveArcade(() -> 0, () -> 0).schedule();
  //System.out.println("No target visible");
}
//   if (tV) {
//     if (!inRange){
//       if(tY<18){
//         if (!currentState.equals("backward")) {
//         if (currentAutoCommand != null) currentAutoCommand.cancel();
//         if (currentBallCommand != null) {
//           currentBallCommand.cancel();
//           currentBallCommand = null;
//           }
//         m_robotContainer.ballSubsystem.runOnce(() -> m_robotContainer.ballSubsystem.stop()).schedule();
//         currentAutoCommand = m_robotContainer.driveSubsystem.driveArcade(() -> -0.4, () -> 0);
//         currentAutoCommand.schedule();
//         currentState = "backward";
//         System.out.println("back");
//          }
//          System.out.println(inRange + "<18");
//       } else if (tY>22){
//          if(!currentState.equals("forward")) {
//             if (currentAutoCommand != null) currentAutoCommand.cancel();
//             if (currentBallCommand != null) {
//               currentBallCommand.cancel();
//               currentBallCommand = null;
//             }
//               m_robotContainer.ballSubsystem.runOnce(() -> m_robotContainer.ballSubsystem.stop()).schedule();
//               currentAutoCommand = m_robotContainer.driveSubsystem
//                   .driveArcade(() -> 0.4, () -> 0);
//               currentAutoCommand.schedule();
//               currentState = "forward";
//               System.out.println("forward");
            
//             }
//             System.out.println(inRange + ">22");
//         } else {
//           inRange = true;
//           System.out.println(inRange);
//         }
//       } else {
//         // Target aligned - stop and shoot
//       if (!currentState.equals("aligned")) {
//       // Only do this ONCE when we first enter the aligned state
//         if (currentAutoCommand != null) {
//         currentAutoCommand.cancel();
//       }
//       if (currentBallCommand != null) {
//       currentBallCommand.cancel();
//       }
//       CommandScheduler.getInstance().cancelAll();
    
//       currentAutoCommand = m_robotContainer.driveSubsystem
//           .driveArcade(() -> 0, () -> 0);
//       currentAutoCommand.schedule();
      
//       mTimer.reset();
//       mTimer.start();
//       currentState = "aligned";
//       System.out.println("stop");
//     }
    
//     // Handle the shooting sequence - but only schedule commands once
//     if (mTimer.get() < 1.0) {
//       if (currentBallCommand == null || currentBallCommand.isFinished()) {
//         currentBallCommand = m_robotContainer.ballSubsystem.spinUpCommand();
//         currentBallCommand.schedule();
//       }
//     } else {
//       if (currentBallCommand == null || currentBallCommand.isFinished()) {
//         currentBallCommand = m_robotContainer.ballSubsystem.launchCommand();
//         currentBallCommand.schedule();
//       }
//     }
//   } 
// } else {
//     // No target visible
//     currentState = "";
//     if (currentAutoCommand != null) {
//       currentAutoCommand.cancel();
//       currentAutoCommand = null;
//     }
//     if (currentBallCommand != null) {
//       currentBallCommand.cancel();
//       currentBallCommand = null;
//     }
//     m_robotContainer.driveSubsystem.driveArcade(() -> 0, () -> 0).schedule();
//   }

}


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
