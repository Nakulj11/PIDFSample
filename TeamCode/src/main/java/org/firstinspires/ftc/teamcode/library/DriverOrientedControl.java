package org.firstinspires.ftc.teamcode.library;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.Moby;

public class DriverOrientedControl {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    public BNO055IMU imu;


    double driveTurn;


    double gamepadXCoordinate;
    double gamepadYCoordinate;
    double gamepadHypot;
    double gamepadDegree;
    double robotDegree;
    double movementDegree;
    double gamepadXControl;
    double gamepadYControl;



    public DriverOrientedControl(){
        imu = Moby.imu.getImu();

        frontLeft = Moby.driveMotors.get(0);
        frontRight = Moby.driveMotors.get(2);
        backRight = Moby.driveMotors.get(3);
        backLeft = Moby.driveMotors.get(1);
    }


    public void run(Gamepad gamepad2, double power){

        //might have to make this negative
        driveTurn = -gamepad2.right_stick_x;
        //driveVertical = -gamepad1.right_stick_y;


        gamepadXCoordinate = gamepad2.left_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = -gamepad2.left_stick_y; //this simply gives our y vaue relative to the driver
        gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
        gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCoordinate, gamepadXCoordinate));

        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        robotDegree = getAngle();
        //gives us the angle our robot is at
        movementDegree = gamepadDegree - robotDegree;


        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors


//        frontRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl)  - gamepadXControl* Math.abs(gamepadXControl)  + driveTurn));
//        backRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn));
//        frontLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl* Math.abs(gamepadXControl) + driveTurn));
//        backLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn));

        frontRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl)  - gamepadXControl* Math.abs(gamepadXControl)  + driveTurn));
        backRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn));
        frontLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl* Math.abs(gamepadXControl) - driveTurn));
        backLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn));


    }



    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



}