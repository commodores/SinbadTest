// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist.WristState;

public class SetWristManualOverride extends CommandBase {
  /** Creates a new SetWristManualOverride. */
  public SetWristManualOverride() {
  addRequirements(RobotContainer.m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Wrist.setState(WristState.MANUAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Wrist.setState(WristState.STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}