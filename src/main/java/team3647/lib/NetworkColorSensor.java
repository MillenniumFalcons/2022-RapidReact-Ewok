// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.lib;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class NetworkColorSensor implements PeriodicSubsystem {
    public final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    public final NetworkTableEntry proxymityEntry;
    public final NetworkTableEntry rawColorsEntry;

    private final double[] kEmptyDoubleThree = new double[3];
    private final int kMaxReadDistance;

    public enum Color {
        NONE("None"),
        RED("Red"),
        GREEN("Green"),
        BLUE("Blue");
        public final String str;

        private Color(String name) {
            str = name;
        }
    }

    private double proximity;
    private double[] rawColors = new double[3];

    private Color currentColor = Color.NONE;
    private Color allianceColor = Color.NONE;

    public NetworkColorSensor(String proximity, String color, int maxReadDistance) {
        this.proxymityEntry = networkTableInstance.getEntry(proximity);
        this.rawColorsEntry = networkTableInstance.getEntry(color);

        kMaxReadDistance = maxReadDistance;
    }

    public double getProximity() {
        return proximity;
    }

    public double[] getRawColors() {
        return rawColors;
    }

    public double getRed() {
        return rawColors[0];
    }

    public double getGreen() {
        return rawColors[1];
    }

    public double getBlue() {
        return rawColors[2];
    }

    public boolean isReadColor() {
        return this.proximity > kMaxReadDistance;
    }

    private Color updateColor() {
        Color result;
        if (getGreen() > getRed() && getGreen() > getBlue() || !isReadColor()) {
            result = Color.NONE;
        } else if (getBlue() > getRed()) {
            result = Color.BLUE;
        } else {
            result = Color.RED;
        }
        return result;
    }

    private boolean isMatchingColor() {
        if (this.allianceColor == this.currentColor) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isCorrectBall() {
        if (isMatchingColor() && isReadColor()) {
            return true;
        } else {
            return false;
        }
    }

    public String getColorAsString() {
        return this.currentColor.str;
    }

    public Color getColor() {
        return this.currentColor;
    }

    @Override
    public void readPeriodicInputs() {
        proximity = proxymityEntry.getDouble(0.0);
        rawColors = rawColorsEntry.getDoubleArray(kEmptyDoubleThree);
        currentColor = updateColor();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "ColorSensor";
    }
}
