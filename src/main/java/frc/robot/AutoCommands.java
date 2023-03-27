package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.AutoBalanceCommand;
import frc.robot.commands.autoCommands.AutoLock;
import frc.robot.commands.autoCommands.Nothing;
import frc.robot.subsystems.*;;

public class AutoCommands {

    private final Swerve swerve;
    public final Map<String, SequentialCommandGroup> autos;
    public final Map<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;

    //Example Multi-Path
    

    public AutoCommands(Swerve swerve) {
        
        this.swerve = swerve;

        //Build Autos
        autos = new HashMap<String, SequentialCommandGroup>();
        eventMap = new HashMap<String, Command>();
        
        /////Do Nothing//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        autos.put("nothing", new SequentialCommandGroup(
          new Nothing()
        ));

        /////Auto Balance Tester//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> AutoBalanceTest = PathPlanner.loadPathGroup("AutoBalanceTest", new PathConstraints(1, 1));
        autos.put("AutoBalanceTest", new SequentialCommandGroup(
            getCommand(AutoBalanceTest),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
        )); 

        //Events////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //eventMap.put("runIntake", new AutoIntake(RobotContainer.m_Intake));
        //eventMap.put("release", new AutoRelease(RobotContainer.m_Intake));
        //eventMap.put("stopIntake", new StopIntake(RobotContainer.m_Intake));
        //eventMap.put("ground", new Ground(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        //eventMap.put("groundUp", new GroundUp(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        //eventMap.put("stow", new Stowed(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        //eventMap.put("high", new High(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        //eventMap.put("mid", new Mid(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        //eventMap.put("groundAuto", new GroundAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist)); 
        //eventMap.put("stowAuto", new StowAuto(RobotContainer.m_Extender, RobotContainer.m_Elevator, RobotContainer.m_Wrist));
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        

    }

    private Command getCommand(List<PathPlannerTrajectory>pathGroup) {                
        
        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXandYControllers, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);

        return autoBuilder.fullAuto(pathGroup);
    }

    

}