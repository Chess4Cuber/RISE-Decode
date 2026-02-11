package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.sensors.ColorSensor;

public class RadahnColorSensor extends ColorSensor {
    public RadahnColorSensor(HardwareMap hardwareMap){
        super(hardwareMap);

        addColorSensor(new String[]{"colorSensor0"});
        addColorSensor(new String[]{"colorSensor1"});
    }
}
