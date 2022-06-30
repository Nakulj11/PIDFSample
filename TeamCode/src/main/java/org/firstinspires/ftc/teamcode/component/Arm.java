package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    //spinner
    private DcMotor arm;

    private ElapsedTime time;

    public enum TurnValueAuto{
        GROUND(0),
        BOTTOM(840),
        MID(1920),
        TOP(3169);

        int ticks;
        TurnValueAuto(int ticks){
            this.ticks = ticks;
        }

        int getTicks(){
            return ticks;
        }
    }

    public enum TurnValueTeleOp{
        GROUND(0),
        BOTTOM(781),
        MID(1897),
        TOP(3009),
        SHARED(1073);

        int ticks;
        TurnValueTeleOp(int ticks){
            this.ticks = ticks;
        }

        int getTicks(){
            return ticks;
        }
    }

    //tetrix: 1440 ticks per revolution
    //andymark: 1120 ticks per rev

    //init
    public void init(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "claw");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void init(HardwareMap hardwareMap, boolean init2){
        arm = hardwareMap.get(DcMotor.class, "claw");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void stopArm(){
        arm.setPower(0);
    }

    public int getTicks(){
        return arm.getCurrentPosition();
    }

    public int getTargetPosition(){
        return arm.getTargetPosition();
    }

    public void setPower(double power){
        arm.setPower(power);
    }

    //takes in input location
    public void moveArm(TurnValueAuto location){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(arm.getCurrentPosition()>location.getTicks()){
            multiplier = -1;
        }
        arm.setTargetPosition(location.getTicks());


        //sets power and mode
        arm.setPower(multiplier * 0.92);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        time = new ElapsedTime();

        //garbage way to determine when to stop mover
//        while(mover.isBusy()){
//
//        }
//        stopMover();
    }

    public double getTime(){
        return time.seconds();
    }

    public void reset(){
        time.reset();
    }

    //takes in input location
    public void moveArmTeleOp(TurnValueTeleOp location){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(arm.getCurrentPosition()>location.getTicks()){
            multiplier = -1;
        }
        arm.setTargetPosition(location.getTicks());

        //sets power and mode
        arm.setPower(multiplier * 0.92);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //garbage way to determine when to stop mover
//        while(mover.isBusy()){
//
//        }
//        stopMover();
    }

    //move claw up by small increments
    public void moveUp(){
        arm.setTargetPosition(arm.getCurrentPosition() + 40    );
        arm.setPower(0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //move claw down by small increments
    public void moveDown(){
        arm.setTargetPosition(arm.getCurrentPosition() - 40);
        arm.setPower(-0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //mover is busy or not
    public boolean isBusy(){
        return arm.isBusy();
    }








}
