package org.firstinspires.ftc.teamcode.mechanisms.RadahnTransfer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.ServoActiveIntake;

public class RadahnServoIntake extends ServoActiveIntake {
    public RadahnServoIntake(HardwareMap hardwareMap){
        super(hardwareMap);

        addOtherRotators(new String[]{"servoIntake"});

    }
}

