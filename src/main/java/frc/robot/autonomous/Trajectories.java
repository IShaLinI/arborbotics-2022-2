package frc.robot.autonomous;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

public class Trajectories {
    
    /**
     * This trajectory is only used to pre-generate all of the trajectories.
     */
    public static Trajectory GENERATE = generateTrajectory(
        1,
        1,
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 0.1, Rotation2d.fromDegrees(0))
        ),
        true,
        "GENERATE"
    );


    public static Trajectory Test = generateTrajectory(
        3,
        6,
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(13), 0, Rotation2d.fromDegrees(0))
        ),
        false,
        "GENERATE"
    );

    /**
     * 
     * Trajectory Generator Function
     * 
     * Author Andrew Card
     * 
     * @param maxVel Max Velocity m/s
     * @param maxAccel Max Accel m/s²
     * @param waypoints Points to create a path to
     * @param reversed Whether you drive forward or backward
     * @param name Label for print purposes
     * @return A Trajectory
     */
    public static Trajectory generateTrajectory(double maxVel, double maxAccel, List<Pose2d> waypoints, boolean reversed, String name) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        config.setReversed(reversed);
        System.out.println("Trajectory '" + name + "' Generated");
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }
}
