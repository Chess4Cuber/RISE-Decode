package org.firstinspires.ftc.baseCode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public abstract class ServoActiveIntake {
    HardwareMap hardwareMap;

    List<CRServo[]> intakeServos = new ArrayList<>();

    public ServoActiveIntake(String[] names, HardwareMap hardwareMap){
        addOtherRotators(names);
    }

    public ServoActiveIntake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void addOtherRotators(String[] names){
        CRServo[] newServos = new CRServo[names.length];
        int flipper = 1;
        for (int i = 0; i < names.length; i++){
            newServos[i] = hardwareMap.get(CRServo.class, names[i]);
            if (flipper > 0) newServos[i].setDirection(CRServo.Direction.FORWARD);
            else newServos[i].setDirection(CRServo.Direction.REVERSE);
            flipper *= -1;
        }
        intakeServos.add(newServos);
    }

    public void setPower(double power){
        for (CRServo servo:intakeServos.get(0)) {
            servo.setPower(power);
        }
    }
    public void setPower(double power, int DoF){
        for (CRServo servo:intakeServos.get(DoF)) {
            servo.setPower(power);
        }
    }
}
