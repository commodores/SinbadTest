// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DigitalInput;

public class Wrist extends SubsystemBase {

	public enum WristState {
		STOW,
		MANUAL,
		POSITION
	}

	public WristState wristState = WristState.STOW;

	public CANSparkMax wrist = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

	public RelativeEncoder encoder = wrist.getEncoder();
	
	private DigitalInput reverseLimitSwitch = new DigitalInput(2);;

	public ArmFeedforward feedForward = new ArmFeedforward(
			WristConstants.wristKs,
			WristConstants.wristKg,
			WristConstants.wristKv);

	private ProfiledPIDController profiledController = new ProfiledPIDController(
			WristConstants.wristKp,
			WristConstants.wristKi,
			WristConstants.wristKd,
			new TrapezoidProfile.Constraints(
        WristConstants.wristMaxVel,
        WristConstants.wristMaxAccel));

	private PIDController controller = new PIDController(
    WristConstants.wristKp,
    WristConstants.wristKi,
    WristConstants.wristKd);


	// neo rotations
	public static double setpointWrist = 0;

	public Wrist() {

		wrist.setIdleMode(IdleMode.kBrake);

		wrist.setInverted(false);

		wrist.setSmartCurrentLimit(50);

		wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
		wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
		wrist.setSoftLimit(SoftLimitDirection.kForward, 8);
		wrist.setSoftLimit(SoftLimitDirection.kReverse, -54);

		// wristA.burnFlash();
		wrist.burnFlash();

		controller.setTolerance(3);
		profiledController.setTolerance(3);
	}

	public void setState(WristState state) {
		wristState = state;
	}

	public WristState getState() {
		return wristState;
	}

	public void setWristOpenLoop(double percent) {
		wrist.set(percent);
	}

	public void setWristClosedLoop(boolean isProfiled) {
		double output = 0;
		if (isProfiled) {
			profiledController.setGoal(setpointWrist);
			output = profiledController.calculate(encoder.getPosition()) +
					feedForward.calculate(Math.toRadians(profiledController.getSetpoint().position),
                                                profiledController.getSetpoint().velocity);
			wrist.set(output);
		} else {
			output = controller.calculate(encoder.getPosition(), setpointWrist) + WristConstants.wristKs;
			wrist.set(MathUtil.clamp(output, -.65, .75));
			// wrist.set(MathUtil.clamp(output, -.2, .3));

		}
	}

	public static double getSetpoint() {
		return setpointWrist;
	}

	public static void setSetpoint(double setpoint) {
		setpointWrist = setpoint;
	}

	public boolean isAtSetpoint(boolean isProfiled) {
		return Math.abs((setpointWrist - encoder.getPosition())) <= 5;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	public void resetEncoder(){
		encoder.setPosition(8);
	}

	public boolean getLimitSwitch(){
		return !reverseLimitSwitch.get();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Wrist Encoder", encoder.getPosition());
		SmartDashboard.putString("Wrist State", String.valueOf(getState()));
		SmartDashboard.putBoolean("Wrist Limit", getLimitSwitch());

		switch (wristState) {
			case STOW:
				setSetpoint(0);
				setWristClosedLoop(false);
				break;
			case MANUAL:
				setWristOpenLoop(-RobotContainer.driverTwo.getLeftY());
				break;
			case POSITION:
				setWristClosedLoop(true);
				break;
		}
	}
}