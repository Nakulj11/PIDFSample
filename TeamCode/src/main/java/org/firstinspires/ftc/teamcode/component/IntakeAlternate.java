package org.firstinspires.ftc.teamcode.component;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeAlternate {
    private CRServo spinner;
    private Servo claw;

    private final double CLOSE =  0;
    private final double OPEN = 0.35;

    //init
    public void init(HardwareMap hardwareMap){

        spinner = hardwareMap.get(CRServo.class, "s1");
        claw =  hardwareMap.get(Servo.class, "s2");

        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(CLOSE);


    }


    //gets position
    public double getPosition(){
        return claw.getPosition();
    }

    //open and close claw
    public void in(){
        spinner.setPower(1);
    }

    public void out(){
        spinner.setPower(-0.5);
    }

    public void stopSpinner(){
        spinner.setPower(0);
    }


    public void open(){
        claw.setPosition(OPEN);
    }

    public void close(){
        claw.setPosition(CLOSE);
    }

    public void out(double power){
        spinner.setPower(-power);
    }



}
