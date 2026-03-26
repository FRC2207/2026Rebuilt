package frc.robot.lib.ObjectVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import frc.robot.current.subsystems.swerveDrive.Drive;
import java.util.function.Consumer;

public class ObjectVisionIODetection implements ObjectVisionIO {
    private Drive swerve;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("VisionData");   
    private StructArraySubscriber<FuelStruct> fuelSub;
    private BooleanSubscriber hopperSubscriber;

    private StructArraySubscriber<Pose2d> kindleWaypoints;
    private Consumer<Pose2d[]> waypointListener;

    public ObjectVisionIODetection(Drive drive) {
        this.swerve = drive;

        // fuelSub = table.getStructArrayTopic("fuel_data", FuelStruct.struct).subscribe(new FuelStruct[]);

        fuelSub = table.getStructArrayTopic("vision_data", FuelStruct.struct).subscribe(new FuelStruct[0]);
        hopperSubscriber = table.getBooleanTopic("hopper_sees_object").subscribe(false);
        kindleWaypoints = table.getStructArrayTopic("DrawnWaypoints", Pose2d.struct).subscribe(new Pose2d[0]);

        inst.addListener(
            table.getStructArrayTopic("DrawnWaypoints", Pose2d.struct),
            java.util.EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                Pose2d[] waypoints = kindleWaypoints.get();

                if (waypointListener != null) {
                    waypointListener.accept(waypoints);
                }
            }
        );
    }

    public void setWaypointListener(Consumer<Pose2d[]> listener) {
        this.waypointListener = listener;
    }
    
    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        FuelStruct[] fuels = fuelSub.get();
        double[] fuelXPoints = new double[fuels.length];
        double[] fuelYPoints = new double[fuels.length];
        for (int i = 0; i < fuels.length; i++) {
            fuelXPoints[i] = fuels[i].x;
            fuelYPoints[i] = fuels[i].y;
        }
        
        inputs.fuelX = fuelXPoints;
        inputs.fuelY = fuelYPoints;
        inputs.hopperSeesObject = hopperSubscriber.get();
        inputs.kindleWaypoints = kindleWaypoints.get();
    }
}