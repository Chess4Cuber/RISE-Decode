package org.firstinspires.ftc.teamcode.mechanisms.intakeSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.ServoActiveIntake;

public class OptimusIntake extends ServoActiveIntake {
    public OptimusIntake(HardwareMap hardwareMap){
        super(hardwareMap);

        addOtherRotators(new String[]{"servo1", "servo2"});

    }
}

