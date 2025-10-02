package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.sensors.ColorSensor;
import org.firstinspires.ftc.baseCode.sensors.TouchSensor;

public class RadahnColorSensors extends ColorSensor {
    public RadahnColorSensors(HardwareMap hardwareMap){
        super(hardwareMap);

        addColorSensor(new String[]{"colorSensor0"});
        addColorSensor(new String[]{"colorSensor1"});
        addColorSensor(new String[]{"colorSensor2"});
    }
}
