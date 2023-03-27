// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ActuateElevatorStow extends CommandBase {
  /** Creates a new ActuateElevatorIdle. */
  double tolerance = 0;
  public ActuateElevatorStow(double tolerance) {
    addRequirements(RobotContainer.m_Elevator); 
    this.tolerance = tolerance;
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Elevator.setState(ElevatorState.STOW);
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
    return RobotContainer.m_Elevator.isAtSetpoint(false, tolerance);
  }
}