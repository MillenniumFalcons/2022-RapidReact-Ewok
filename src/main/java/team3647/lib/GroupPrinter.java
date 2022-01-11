package team3647.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/** Prints to SmartDashboard every loop */
public class GroupPrinter implements Subsystem {
    private static final GroupPrinter INSTANCE = new GroupPrinter();

    public static GroupPrinter getInstance() {
        return INSTANCE;
    }

    private final Map<String, Supplier<Double>> doublePrint = new HashMap<>();
    private final Map<String, Supplier<String>> stringPrint = new HashMap<>();
    private final Map<String, Supplier<Boolean>> boolPrint = new HashMap<>();

    private GroupPrinter() {}

    public void addDouble(String key, Supplier<Double> func) {
        doublePrint.put(key, func);
        SmartDashboard.putNumber(key, func.get());
    }

    public void addString(String key, Supplier<String> func) {
        stringPrint.put(key, func);
        SmartDashboard.putString(key, func.get());
    }

    public void addBoolean(String key, Supplier<Boolean> func) {
        boolPrint.put(key, func);
        SmartDashboard.putBoolean(key, func.get());
    }

    @Override
    public void periodic() {
        for (var entry : doublePrint.entrySet()) {
            SmartDashboard.putNumber(entry.getKey(), entry.getValue().get());
        }
        for (var entry : stringPrint.entrySet()) {
            SmartDashboard.putString(entry.getKey(), entry.getValue().get());
        }
        for (var entry : boolPrint.entrySet()) {
            SmartDashboard.putBoolean(entry.getKey(), entry.getValue().get());
        }
    }
}
