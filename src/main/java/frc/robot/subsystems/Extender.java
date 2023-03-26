// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Extender extends SubsystemBase {

  private final CANSparkMax extenderMotor;

  private SparkMaxPIDController extenderPIDController;
  private RelativeEncoder extenderEncoder;
  private DigitalInput reverseLimitSwitch;
  //private final TimeOfFlight extenderDistanceSensor;
  private final double forwardLimit, reverseLimit;
  
  double kP = Constants.ExtenderConstants.extenderKP,
    kI = Constants.ExtenderConstants.extenderKI,
    kD = Constants.ExtenderConstants.extenderKD,
    kIz = Constants.ExtenderConstants.extenderKIz,
    kFF = Constants.ExtenderConstants.extenderKFF, 
    kMinOutput = Constants.ExtenderConstants.extenderKMinOutput,
    kMaxOutput = Constants.ExtenderConstants.extenderKMaxOutput,
    minVel = Constants.ExtenderConstants.extenderMinVel,
    maxVel = Constants.ExtenderConstants.extenderMaxVel,
    maxAcc = Constants.ExtenderConstants.extenderMaxAcc,
    allowedErr = Constants.ExtenderConstants.extenderAllowedErr;

  
  /** Creates a new Elevator. */
  public Extender() {
    extenderMotor = new CANSparkMax(ExtenderConstants.extenderMotorID, MotorType.kBrushless);

    //extenderDistanceSensor = new TimeOfFlight(ExtenderConstants.extenderDistanceSensorID);  

    extenderMotor.restoreFactoryDefaults();
    extenderMotor.setSmartCurrentLimit(80);
    extenderMotor.setIdleMode(IdleMode.kBrake);
    reverseLimit = Units.inchesToMeters(.25)*ExtenderConstants.KExtenderMetersToNeoRotationsFactor;
    forwardLimit = Units.inchesToMeters(20)*ExtenderConstants.KExtenderMetersToNeoRotationsFactor;

    extenderMotor.setSoftLimit(SoftLimitDirection.kForward,((float)forwardLimit));
    extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, ((float)reverseLimit));

    extenderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extenderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    extenderPIDController = extenderMotor.getPIDController();
    extenderEncoder = extenderMotor.getEncoder();
    reverseLimitSwitch = new DigitalInput(0);

    // set PID coefficients
    extenderPIDController.setP(kP);
    extenderPIDController.setI(kI);
    extenderPIDController.setD(kD);
    extenderPIDController.setIZone(kIz);
    extenderPIDController.setFF(kFF);
    extenderPIDController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    extenderPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    extenderPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    extenderPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    extenderPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

   // seedEncoder();
  }

  public void setPosition(double targetPosition){
    extenderPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void manualExtender(double speed){
    extenderMotor.set(speed);
  }

  public double getOutputCurrent() {
    return extenderMotor.getOutputCurrent();
  }

  public double getPosition() {
    return Units.metersToInches(extenderEncoder.getPosition()/ExtenderConstants.KExtenderMetersToNeoRotationsFactor);
  }

  public void resetEncoder(){
    extenderMotor.getEncoder().setPosition(0);
  }

  public void setEncoder(double position){
    extenderMotor.getEncoder().setPosition(position);
  }


  public boolean getLimitSwitch(){
    return !reverseLimitSwitch.get();
  }

  //public double getDistanceSensor(){
  //  return Units.metersToInches((extenderDistanceSensor.getRange()*.001)-.0127);
  //}

  //public void seedEncoder(){
  //  extenderEncoder.setPosition(Units.inchesToMeters(getDistanceSensor()*ExtenderConstants.KExtenderMetersToNeoRotationsFactor));
  //}



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extender Current", getOutputCurrent());
    SmartDashboard.putNumber("Extender Position", getPosition());
    //SmartDashboard.putNumber("Extender Distance Sensor Position", getDistanceSensor());
    SmartDashboard.putBoolean("Extend Reverse Limit Switch", getLimitSwitch());
  
  }
  
}