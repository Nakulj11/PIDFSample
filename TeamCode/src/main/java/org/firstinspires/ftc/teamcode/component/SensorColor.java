package org.firstinspires.ftc.teamcode.component;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SensorColor {

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private float colorValues[] = {0F,0F,0F};

    private float lowerLimitBrick[] = {6F,0F,5F};
    private float upperLimitBrick[] = {9F,2F,8F};

    private float lowerLimitBall[] = {7F,2F,7F};
    private float upperLimitBall[] = {11F,4F,12F};

    private final double DISTANCE = 3;

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        colorSensor.enableLed(false);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
    }

    public boolean matches(){
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, colorValues);
        boolean matches = true;
        for(int i=0;i<3;i++){
            if((colorValues[i]<lowerLimitBrick[i]||colorValues[i]>upperLimitBrick[i])&&(colorValues[i]<lowerLimitBall[i]||colorValues[i]>upperLimitBall[i])){
                matches = false;
            }

        }
        return matches;
    }

    public float getBlue(){
        return colorSensor.blue();
    }

    public float getRed(){
        return colorSensor.red();
    }

    public float getGreen(){
        return colorSensor.green();
    }

    public boolean intakeSuccessful(){
        return distanceSensor.getDistance(DistanceUnit.CM)<DISTANCE;
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

}