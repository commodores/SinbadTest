package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.wrist.SetWristManualOverride;
import frc.robot.commands.elevator.*;
import frc.robot.commands.extender.ActuateExtenderToSetpoint;
import frc.robot.commands.extender.SetExtenderManualOverride;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    public final static Intake m_Intake = new Intake();
    public final static Extender m_Extender = new Extender();
    public final static Elevator m_Elevator = new Elevator();
    public final static Wrist m_Wrist = new Wrist();
  
    private final SendableChooser<SequentialCommandGroup> autoChooser;
    private final AutoCommands autos;    

    /* Controllers */
    public final static XboxController driver = new XboxController(0);
    public final static XboxController driverTwo = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons Controller 1 */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);    
    private final JoystickButton extendArm = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton retractArm = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton raiseElevator = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton lowerElevator = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton raiseWrist = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton lowerWrist = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton homeEverything = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    

    /* Driver Buttons Controller 2 */
    private final JoystickButton intake  = new JoystickButton(driverTwo, XboxController.Button.kRightBumper.value);
    private final JoystickButton release = new JoystickButton(driverTwo, XboxController.Button.kLeftBumper.value);  
    private final JoystickButton stow = new JoystickButton(driverTwo, XboxController.Button.kB .value);
    private final JoystickButton ground = new JoystickButton(driverTwo, XboxController.Button.kA.value);
    private final JoystickButton mid = new JoystickButton(driverTwo, XboxController.Button.kX.value);
    private final JoystickButton high = new JoystickButton(driverTwo, XboxController.Button.kY.value);
    private final JoystickButton shelf = new JoystickButton(driverTwo, XboxController.Button.kStart.value);
    private final JoystickButton groundUp = new JoystickButton(driverTwo, XboxController.Button.kBack.value);
    private final JoystickButton singleStation = new JoystickButton(driverTwo, XboxController.Button.kLeftStick.value);
    private final JoystickButton groundCube = new JoystickButton(driverTwo, XboxController.Button.kRightStick.value);


    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
   public RobotContainer() {
      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis)*.75, 
          () -> robotCentric.getAsBoolean()
        )
      );

      autos = new AutoCommands(s_Swerve);
      autoChooser = new SendableChooser<>();
    
      Set<String> keys = autos.autos.keySet();
      autoChooser.setDefaultOption((String) keys.toArray()[1], autos.autos.get(keys.toArray()[1]));
      //keys.remove((String) keys.toArray()[0]);
    
      for (String i : autos.autos.keySet()) {
        autoChooser.addOption(i, autos.autos.get(i));
      }

      SmartDashboard.putData("Auto Selector", autoChooser);
    
      // Configure the button bindings
      configureButtonBindings();
}

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver 1 Buttons */

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));//----Y Button
        
        //extendArm.onTrue(new ManualExtender(m_Extender, .7));//----Right Bumper
        //extendArm.onFalse(new ManualExtender(m_Extender, 0).withTimeout(0.1));
        //retractArm.onTrue(new ManualExtender(m_Extender, -.7));//----Left Bumper
        //retractArm.onFalse(new ManualExtender(m_Extender, 0).withTimeout(0.1));
       
        //raiseElevator.onTrue(new ManualElevator(m_Elevator, .5));//----Start Button
        //raiseElevator.onFalse(new ManualElevator(m_Elevator, 0).withTimeout(0.1));
        //lowerElevator.onTrue(new ManualElevator(m_Elevator, -.5));//----Back Button
        //lowerElevator.onFalse(new ManualElevator(m_Elevator, 0).withTimeout(0.1));

        //raiseWrist.onTrue(new ManualWrist(m_Wrist, .5));//-------A Button
        //raiseWrist.onFalse(new ManualWrist(m_Wrist, 0));
        //lowerWrist.onTrue(new ManualWrist(m_Wrist, -.5));//-------B Button
        //lowerWrist.onFalse(new ManualWrist(m_Wrist, 0));

        //homeEverything.onTrue(new Home(m_Extender, m_Elevator, m_Wrist, m_Intake));//-------right Stick button

        
        /* Driver 2 Buttons */        

        //intake.onTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(.8)));//----Right Bumper
        //intake.onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(.04)));
        //release.onTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(-.8)));//----Left Bumper
        //release.onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(.04)));
        
        //stow.onTrue(new Stowed(m_Extender, m_Elevator, m_Wrist));//----B Button
        //ground.onTrue(new Ground(m_Extender, m_Elevator, m_Wrist));//----A Button
        //groundUp.onTrue(new GroundUp(m_Extender, m_Elevator, m_Wrist));//------Back Button
        //mid.onTrue(new Mid(m_Extender, m_Elevator, m_Wrist));//----X Button
        //high.onTrue(new High(m_Extender, m_Elevator, m_Wrist));//----Y Button
        //shelf.onTrue(new Shelf(m_Extender, m_Elevator, m_Wrist));//----Start Button       
        //singleStation.onTrue(new SingleStation(m_Extender, m_Elevator, m_Wrist));//---Left Stick Button
        //groundCube.onTrue(new GroundAuto(m_Extender, m_Elevator, m_Wrist));//---Right Stick Button

        shelf.toggleOnTrue(new SetElevatorManualOverride());
        shelf.toggleOnTrue(new SetExtenderManualOverride());
        shelf.toggleOnTrue(new SetWristManualOverride());


        high.onTrue(new ActuateElevatorToSetpoint(6, 1));
        mid.onTrue(new ActuateExtenderToSetpoint(6, 1));

        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }
}
