package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {

    //spinner
    DcMotor spinningMotor;

    //speed constant
    private final double SPEED = 0.9;

    public void init(HardwareMap hardwareMap){
        spinningMotor = hardwareMap.get(DcMotor.class, "spin");

//        spinningMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //methods to set spinner to powers
    public void spin(){
        spinningMotor.setPower(SPEED);
    }

    public void stop(){
        spinningMotor.setPower(0);
    }

    public void reverse(){
        spinningMotor.setPower(-SPEED);
    }




}