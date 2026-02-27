package frc.robot.lib.ObjectVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectVision extends SubsystemBase {
    private ObjectVisionIO io;

    public ObjectVision(ObjectVisionIO io) {
        this.io = io;
    }

    public Command getPath() {
        return io.getPath();
    }

    public Command getDynamicPath() {
        return io.getDynamicPath();
    }

    public Boolean hopperSeesObject() {
        return io.hopperSeesObject();
    }
}
