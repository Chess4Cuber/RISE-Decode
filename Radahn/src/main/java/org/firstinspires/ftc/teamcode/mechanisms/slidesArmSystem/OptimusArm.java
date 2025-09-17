package org.firstinspires.ftc.teamcode.mechanisms.slidesArmSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.baseCode.hardware.arms.ServoArm;

public class OptimusArm extends ServoArm {
    public OptimusArm(HardwareMap hardwareMap){
        super(hardwareMap);

        addDegreeOfFreedom(new String[]{"armServoLeft", "armServoRight"});
        addDegreeOfFreedom(new String[]{"rotator"});
        addDegreeOfFreedom(new String[]{"wrist"});
    }
}
