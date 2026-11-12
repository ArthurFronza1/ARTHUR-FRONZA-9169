
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.TankdriveSub;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TankdriveCMD extends Command {

  private TankdriveSub drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankdriveCMD(TankdriveSub drive) {

    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rt = RobotContainer.controller.getRightTriggerAxis();
    double lt = RobotContainer.controller.getLeftTriggerAxis();
    double X = RobotContainer.controller.getLeftX();

    double velocidade = MathUtil.applyDeadband(lt, 0.1) -
        MathUtil.applyDeadband(rt, 0.1);

    double rotacao = MathUtil.applyDeadband(X, 0.1);

    drive.arcadeDrive(velocidade, rotacao);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
