package frc.robot.lib.ObjectVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectVision extends SubsystemBase {
    private ObjectVisionIO io;

    public ObjectVision(ObjectVisionIO io) {
        this.io = io;
    }

    public Trajectory getPath() {
        return io.getPath();
    }

    public Pose2d[] subsystemData(String subsystemName) {
        return io.subsystemData(subsystemName);
    }

    public Boolean seesObject() {
        return io.seesObject();
    }
}
