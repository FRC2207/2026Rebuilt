package frc.robot.lib.ObjectVision;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import frc.robot.current.subsystems.swerveDrive.Drive;

public class ObjectVisionIODetection implements ObjectVisionIO {
    private Drive swerve;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("VisionData");   
    private StructArraySubscriber<FuelStruct> fuelSub;
    private BooleanSubscriber hopperSubscriber;

    public ObjectVisionIODetection(Drive drive) {
        this.swerve = drive;

        // fuelSub = table.getStructArrayTopic("fuel_data", FuelStruct.struct).subscribe(new FuelStruct[]);

        fuelSub = table.getStructArrayTopic("vision_data", FuelStruct.struct).subscribe(new FuelStruct[0]);
        hopperSubscriber = table.getBooleanTopic("hopper_sees_object").subscribe(false);
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
    }
}