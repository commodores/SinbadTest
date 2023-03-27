// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class ActuateWristToSetpoint extends CommandBase {

  double setpoint = 0;
  double tolerance = 0;
  public ActuateWristToSetpoint(double setpoint, double tolerance) {
    addRequirements(RobotContainer.m_Wrist);
    this.setpoint = setpoint;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Wrist.setSetpoint(setpoint);
    RobotContainer.m_Wrist.setState(WristState.POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.wrist.setState(WristState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_Wrist.isAtSetpoint(false, tolerance);
  }
}
