// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants; 

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private SparkMaxConfig globalconfig;
  private SparkMaxConfig Rlider;
  private SparkMaxConfig Rfollow;
  private SparkMaxConfig Llider;
  private SparkMaxConfig Lfollow;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    globalconfig = new SparkMaxConfig();
    Rlider = new SparkMaxConfig();
    Rfollow = new SparkMaxConfig();
    Llider = new SparkMaxConfig();
    Lfollow = new SparkMaxConfig();
  }

  @Override
  public void robotInit() {
      
    globalconfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);


    Rlider.apply(globalconfig);

    m_robotContainer.drive.SparkRlider.configure(
      Rlider, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );


    Rfollow.apply(globalconfig).follow(OperatorConstants.IDSparkRlider, false);

    m_robotContainer.drive.SparkRfollow.configure(
      Rfollow, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    


    Llider.apply(globalconfig);

    m_robotContainer.drive.SparkLlider.configure(
      Llider, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    
    Lfollow.apply(globalconfig).follow(OperatorConstants.IDSparkLlider, false);

    m_robotContainer.drive.SparkLfollow.configure(
      Lfollow, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
