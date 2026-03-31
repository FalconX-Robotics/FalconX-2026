package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;

public class RightClimbAuto {

    PathPlannerAuto auto;
    PathPlannerPath path;
    // Waypoint startWaypoint;
    // Waypoint waypoint2;
    // Waypoint endWaypoint;
    PathConstraints pathConstraints;


    public RightClimbAuto(RobotContainer robotContainer) {
        // startWaypoint = new Waypoint(null, new Translation2d(2, 2), null);
        // waypoint2 = new Waypoint(null, null, null);
        // endWaypoint = new Waypoint(null, null, null);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(3.63796005706134, 0.7809272467902997, Rotation2d.fromDegrees(180)),
        new Pose2d(1.322211111111111, 3.246, Rotation2d.fromDegrees(180)),
        new Pose2d(0.38674444444444445, 3.3723777777777775, Rotation2d.fromDegrees(180))
);        

        pathConstraints = robotContainer.subsystems.swerve.pathConstraints;

        path = new PathPlannerPath(waypoints,
                pathConstraints,
                 new IdealStartingState(0, null),
                 new GoalEndState(0, null));
        
        
    }
    
}