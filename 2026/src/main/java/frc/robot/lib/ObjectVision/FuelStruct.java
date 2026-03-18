package frc.robot.lib.ObjectVision;
import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class FuelStruct implements StructSerializable {
    public double x;
    public double y;

    public static final FuelStructDef struct = new FuelStructDef();

    public FuelStruct(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public FuelStruct() {}

    public static class FuelStructDef implements Struct<FuelStruct> {
        @Override public Class<FuelStruct> getTypeClass() { return FuelStruct.class; }
        @Override public String getTypeString() { return "struct:Fuel"; }
        @Override public int getSize() { return Double.BYTES * 2; }
        @Override public String getSchema() { return "double x;double y;"; }
        @Override public String getTypeName() { return "Fuel"; }

        @Override
        public FuelStruct unpack(ByteBuffer bb) {
            FuelStruct f = new FuelStruct();
            f.x = bb.getDouble();
            f.y = bb.getDouble();
            return f;
        }

        @Override
        public void pack(ByteBuffer bb, FuelStruct val) {
            bb.putDouble(val.x);
            bb.putDouble(val.y);
        }
    }
}