// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorMotor;

  private SparkMaxPIDController elevatorPIDController;
  private RelativeEncoder elevatorEncoder;
  // private final TimeOfFlight extenderDistanceSensor;
  private final DigitalInput reverseLimitSwitch;
  private final ElevatorFeedforward m_feedforward;

  private static double deltaTime = 0.02;

  private TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(10,10);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(0.15, 0.0, 0.0, m_constraints, deltaTime);
  

  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new CANSparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    m_feedforward = new ElevatorFeedforward(ElevatorConstants.elevatorKs, ElevatorConstants.elevatorKg, ElevatorConstants.elevatorKv, ElevatorConstants.elevatorKa);


    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setSmartCurrentLimit(80);
    elevatorMotor.setIdleMode(IdleMode.kBrake);

    elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 18);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    
    // initialze PID controller and encoder objects
    elevatorPIDController = elevatorMotor.getPIDController();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorEncoder.setPositionConversionFactor(ElevatorConstants.elevatorInchesToNeoRotationsFactor);
    elevatorEncoder.setVelocityConversionFactor(ElevatorConstants.elevatorInchesVelocityToNeoRotationsFactor);


    reverseLimitSwitch = new DigitalInput(1);
    
  }




  @Override
  public void periodic() {

    
  }

  
}