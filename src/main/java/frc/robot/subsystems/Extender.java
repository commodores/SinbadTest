// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DigitalInput;

public class Extender extends SubsystemBase {

	public enum ExtenderState {
		STOW,
		MANUAL,
		POSITION
	}

	public ExtenderState extenderState = ExtenderState.STOW;

	public CANSparkMax extender = new CANSparkMax(ExtenderConstants.extenderMotorID, MotorType.kBrushless);

	public RelativeEncoder encoder = extender.getEncoder();
	
	private DigitalInput reverseLimitSwitch = new DigitalInput(0);;

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			ExtenderConstants.extenderKs,
			ExtenderConstants.extenderKg,
			ExtenderConstants.extenderKv);

	private ProfiledPIDController profiledController = new ProfiledPIDController(
			ExtenderConstants.extenderKp,
			ExtenderConstants.extenderKi,
			ExtenderConstants.extenderKd,
			new TrapezoidProfile.Constraints(
        ExtenderConstants.extenderMaxVel,
        ExtenderConstants.extenderMaxAccel));

	private PIDController controller = new PIDController(
    ExtenderConstants.extenderKp,
    ExtenderConstants.extenderKi,
    ExtenderConstants.extenderKd);


	// neo rotations
	public static double setpointExtender = 0;

	public Extender() {

		extender.setIdleMode(IdleMode.kBrake);

		extender.setInverted(false);

		extender.setSmartCurrentLimit(50);

		extender.enableSoftLimit(SoftLimitDirection.kForward, true);
		extender.enableSoftLimit(SoftLimitDirection.kReverse, true);
		extender.setSoftLimit(SoftLimitDirection.kForward, 18);
		extender.setSoftLimit(SoftLimitDirection.kReverse, 0);

		// extenderA.burnFlash();
		extender.burnFlash();

    	encoder.setPositionConversionFactor(ExtenderConstants.extenderInchesToNeoRotationsFactor);

		controller.setTolerance(3);
		profiledController.setTolerance(3);
	}

	public void setState(ExtenderState state) {
		extenderState = state;
	}

	public ExtenderState getState() {
		return extenderState;
	}

	public void setExtenderOpenLoop(double percent) {
		extender.set(percent);
	}

	public void setExtenderClosedLoop(boolean isProfiled) {
		double output = 0;
		if (isProfiled) {
			profiledController.setGoal(setpointExtender);
			output = profiledController.calculate(encoder.getPosition()) +
					feedForward.calculate(profiledController.getSetpoint().velocity);
			extender.set(output);
		} else {
			output = controller.calculate(encoder.getPosition(), setpointExtender) + ExtenderConstants.extenderKs;
			extender.set(MathUtil.clamp(output, -.65, .75));
			// extender.set(MathUtil.clamp(output, -.2, .3));

		}
	}

	public static double getSetpoint() {
		return setpointExtender;
	}

	public static void setSetpoint(double setpoint) {
		setpointExtender = setpoint;
	}

	public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
		return Math.abs((setpointExtender - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	public void resetEncoder(){
		encoder.setPosition(0);
	}

	public boolean getLimitSwitch(){
		return !reverseLimitSwitch.get();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Extender Encoder", encoder.getPosition());
		SmartDashboard.putString("Extender State", String.valueOf(getState()));
		SmartDashboard.putBoolean("Extender Limit", getLimitSwitch());

		switch (extenderState) {
			case STOW:
				setSetpoint(0);
				setExtenderClosedLoop(false);
				break;
			case MANUAL:
				setExtenderOpenLoop(-RobotContainer.driverTwo.getLeftX());
				break;
			case POSITION:
				setExtenderClosedLoop(true);
				break;
		}
	}
}