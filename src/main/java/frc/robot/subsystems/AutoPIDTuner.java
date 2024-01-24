import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoPIDTuner extends SubsystemBase {
    private double P, I, D;

    private final NetworkTableEntry pEntry;
    private final NetworkTableEntry iEntry;
    private final NetworkTableEntry dEntry;

    public AutoPIDTuner() {
        // Access the NetworkTable for PID values
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        NetworkTable table = ntinst.getTable("AutoPIDValues");
        pEntry = table.getEntry("P");
        iEntry = table.getEntry("I");
        dEntry = table.getEntry("D");

        // Set initial PID values from NetworkTables
        updatePIDValues();
    }

    @Override
    public void periodic() {
        // Update PID values periodically
        updatePIDValues();
    }

    private void updatePIDValues() {
        // Retrieve PID values from NetworkTables
        P = pEntry.getDouble(0.0);
        I = iEntry.getDouble(0.0);
        D = dEntry.getDouble(0.0);
    }

    // Getters for PID values
    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }
}

