package org.firstinspires.ftc.baseCode.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public abstract class TouchSensor {
    HardwareMap hardwareMap;
    List<DigitalChannel[]> touchSensors = new ArrayList<>();

    public TouchSensor(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void addTouchSensor(String[] names){
        DigitalChannel[] newSensors = new DigitalChannel[names.length];
        int flipper = 1;
        for (int i = 0; i < names.length; i++){
            newSensors[i] = hardwareMap.get(DigitalChannel.class, names[i]);

            if (flipper > 0){
                newSensors[i].setMode(DigitalChannel.Mode.INPUT);
                newSensors[i].setState(false);
            }
            else{
                newSensors[i].setMode(DigitalChannel.Mode.INPUT);
                newSensors[i].setState(false);
            }

            flipper *= -1;
        }
        touchSensors.add(newSensors);
    }

    public boolean getTouchSensor(int Num){
        return touchSensors.get(Num)[0].getState();
    }

    public void setTouchSensor(int Num){
        touchSensors.get(Num)[0].setState(false);
    }


}
