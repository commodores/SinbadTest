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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION
	}

	public ElevatorState elevatorState = ElevatorState.IDLE;

	public CANSparkMax elevator = new CANSparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

	public RelativeEncoder encoder = elevator.getEncoder();

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			ElevatorConstants.elevatorKs,
			ElevatorConstants.elevatorKg,
			ElevatorConstants.elevatorKv);

	private ProfiledPIDController profiledController = new ProfiledPIDController(
			ElevatorConstants.elevatorKp,
			ElevatorConstants.elevatorKi,
			ElevatorConstants.elevatorKd,
			new TrapezoidProfile.Constraints(
        ElevatorConstants.elevatorMaxVel,
        ElevatorConstants.elevatorMaxAccel));

	private PIDController controller = new PIDController(
    ElevatorConstants.elevatorKp,
    ElevatorConstants.elevatorKi,
    ElevatorConstants.elevatorKd);


	// neo rotations
	public static double setpointElevator = 0;

	public Elevator() {

		elevator.setIdleMode(IdleMode.kBrake);

		elevator.setInverted(false);

		elevator.enableSoftLimit(SoftLimitDirection.kForward, true);
		elevator.enableSoftLimit(SoftLimitDirection.kReverse, true);
		elevator.setSoftLimit(SoftLimitDirection.kForward, 18);
		elevator.setSoftLimit(SoftLimitDirection.kReverse, 0);

		// elevatorA.burnFlash();
		elevator.burnFlash();

    encoder.setPositionConversionFactor(ElevatorConstants.elevatorInchesToNeoRotationsFactor);

		controller.setTolerance(3);
		profiledController.setTolerance(3);
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setElevatorOpenLoop(double percent) {
		elevator.set(percent);
	}

	public void setElevatorClosedLoop(boolean isProfiled) {
		double output = 0;
		if (isProfiled) {
			profiledController.setGoal(setpointElevator);
			output = profiledController.calculate(encoder.getPosition()) +
					feedForward.calculate(profiledController.getSetpoint().velocity);
			elevator.set(output);
		} else {
			output = controller.calculate(encoder.getPosition(), setpointElevator) + ElevatorConstants.elevatorKs;
			elevator.set(MathUtil.clamp(output, -.65, .75));
			// elevator.set(MathUtil.clamp(output, -.2, .3));

		}
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
		return Math.abs((setpointElevator - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
		SmartDashboard.putString("Elevator State", String.valueOf(getState()));

		switch (elevatorState) {
			case IDLE:
				setSetpoint(0);
				setElevatorClosedLoop(false);
				break;
			case MANUAL:
				setElevatorOpenLoop(RobotContainer.driverTwo.getLeftY());
				break;
			case POSITION:
				setElevatorClosedLoop(false);
				break;
		}
	}
}