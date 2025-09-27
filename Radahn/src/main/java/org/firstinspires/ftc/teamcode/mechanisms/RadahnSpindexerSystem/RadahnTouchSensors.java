package org.firstinspires.ftc.teamcode.mechanisms.RadahnSpindexerSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.sensors.TouchSensor;

public class RadahnTouchSensors extends TouchSensor {
    public RadahnTouchSensors(HardwareMap hardwareMap){
        super(hardwareMap);

        addTouchSensor(new String[]{"touchSensor0"});
        addTouchSensor(new String[]{"touchSensor1"});
        addTouchSensor(new String[]{"touchSensor2"});
    }
}
