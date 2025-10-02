package org.firstinspires.ftc.baseCode.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;
import java.util.List;

public abstract class ColorSensor {
    HardwareMap hardwareMap;
    List<NormalizedColorSensor[]> colorSensors = new ArrayList<>();


    public enum ColorStates{
        GREEN,
        PURPLE
    }

    public ColorSensor(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void addColorSensor(String[] names){
        NormalizedColorSensor[] newSensors = new NormalizedColorSensor[names.length];
        int flipper = 1;
        for (int i = 0; i < names.length; i++){
            newSensors[i] = hardwareMap.get(NormalizedColorSensor.class, names[i]);

            flipper *= -1;
        }
        colorSensors.add(newSensors);
    }

    public NormalizedRGBA getColor(int num){
        return colorSensors.get(num)[0].getNormalizedColors();
    }


    public float getRed(int num){
        return getColor(num).red;
    }

    public float getGreen(int num){
        return getColor(num).green;
    }

    public float getBlue(int num){
        return getColor(num).blue;
    }

    public float getAlpha(int num){
        return getColor(num).alpha;
    }


    public boolean seesColor(int num, float r, float g, float b, float a){
        NormalizedRGBA color = getColor(num);
        return (Math.abs(color.red - r) < a &&
                Math.abs(color.green - g) < a &&
                Math.abs(color.blue - b) < a);
    }



}
