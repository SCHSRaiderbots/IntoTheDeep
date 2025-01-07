package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SampleDetector {
    public enum SAMPLE_COLOR {NAUGHT, BLUE, RED, YELLOW }

    /** The colorSensor */
    NormalizedColorSensor colorSensor;

    float gain = 2;

    /** HSV values from the sensor */
    final float[] hsvValues = new float[3];


    public SampleDetector(HardwareMap hardwareMap) {
        // get the color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // turn on the light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // set the gain
        colorSensor.setGain(gain);
    }

    public SAMPLE_COLOR getColor() {
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        // check the distance
        if (colorSensor instanceof DistanceSensor) {
            // distance in centimeters
            double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

            // make sure the distance is less than some value
            if (dist > 6.0) {
                // sample too far away
                return SAMPLE_COLOR.NAUGHT;
            }
        }

        //  H S V are 0, 1, 2
        if (hsvValues[1] < 0.5) {
            // saturation is too low for a good reading
            return SAMPLE_COLOR.NAUGHT;
        }

        // now dispatch on hue
        if (hsvValues[0] < 45.0) {
            return SAMPLE_COLOR.RED;
        } else if (hsvValues[0] < 120.0) {
            return SAMPLE_COLOR.YELLOW;
        } else {
            return SAMPLE_COLOR.BLUE;
        }

    }
}
