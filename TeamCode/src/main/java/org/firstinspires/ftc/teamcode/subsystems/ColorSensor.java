package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    private NormalizedColorSensor colorSensor;
    private Telemetry telemetry;

    // We will track the last 15 readings for a quick but reliable confidence check
    private final int SAMPLE_SIZE = 15;
    private int consecutiveDetections = 0;

    // How close a sample needs to be (in CM) before we care about its color
    // You may need to tune this depending on how deep the sensor is mounted in your intake!
    private final double DETECTION_DISTANCE_CM = 3.5;

    // Possible states based on the hue ranges you provided
    public enum SampleColor { NONE, TARGET_150_170, TARGET_230_250 }
    public SampleColor lastDetected = SampleColor.NONE;

    public void init(HardwareMap hardwareMap, Telemetry tele) {
        this.telemetry = tele;

        // Make sure the name here matches your configuration on the Driver Hub!
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colour_sensor");

        // Gain amplifies the light. 2.0 is a great starting point for FTC.
        colorSensor.setGain(2.0f);
    }

    /**
     * Call this inside your loop. It returns true ONLY when it is highly confident
     * that a new sample has fully entered the intake based on the requested hues.
     */
    public boolean detectNewSample() {
        // 1. Check if anything is close enough to the sensor.
        // If the intake is empty, reset our counters and return false.
        if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > DETECTION_DISTANCE_CM) {
            consecutiveDetections = 0;
            lastDetected = SampleColor.NONE;
            return false;
        }

        // 2. Something is physically close! Get the Hue.
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        // 3. Determine the color based on your specific Hue ranges
        SampleColor detectedThisLoop = SampleColor.NONE;

        if (hue >= 150 && hue <= 170) {
            detectedThisLoop = SampleColor.TARGET_150_170;
        } else if (hue >= 230 && hue <= 250) {
            detectedThisLoop = SampleColor.TARGET_230_250;
        }

        // 4. Confidence Check
        if (detectedThisLoop != SampleColor.NONE && detectedThisLoop == lastDetected) {
            consecutiveDetections++;
        } else {
            consecutiveDetections = 0; // Reset if the color flickers or is a random piece of tape
            lastDetected = detectedThisLoop;
        }

        // Printouts for the Driver Station so you can debug and tune
        telemetry.addData("Sensor Distance (cm)", "%.2f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        telemetry.addData("Sensor Hue", "%.1f", hue);
        telemetry.addData("Seeing Color", lastDetected);
        telemetry.addData("Confidence Count", consecutiveDetections + "/" + SAMPLE_SIZE);

        // 5. If we have seen the SAME color for 15 loops in a row, trigger the spindexer!
        if (consecutiveDetections == SAMPLE_SIZE) {
            // We return true exactly ONCE when the count hits the sample size.
            // If the piece stays in the intake, the count goes to 16, 17, 18, so it won't re-trigger.
            return true;
        }

        return false;
    }
}