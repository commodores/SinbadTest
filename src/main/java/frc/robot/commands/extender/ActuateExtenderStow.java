// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Extender.ExtenderState;

public class ActuateExtenderStow extends CommandBase {
  /** Creates a new ActuateExtenderIdle. */
  double tolerance = 0;
  public ActuateExtenderStow(double tolerance) {
    addRequirements(RobotContainer.m_Extender); 
    this.tolerance = tolerance;
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Extender.setState(ExtenderState.STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_Extender.isAtSetpoint(false, tolerance);
  }
}