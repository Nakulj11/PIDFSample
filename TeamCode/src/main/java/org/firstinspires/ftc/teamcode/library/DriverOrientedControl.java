package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    //double driveVertical;
    //double driveHorizontal;

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
        driveTurn = gamepad2.right_stick_x;
        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        gamepadXCoordinate = gamepad2.left_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = -gamepad2.left_stick_y; //this simply gives our y vaue relative to the driver
        gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
        //finds just how much power to give the robot based on how much x and y given by gamepad
        //range.clip helps us keep our power within positive 1
        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
        gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        robotDegree = getAngle();
        //gives us the angle our robot is at
        movementDegree = gamepadDegree - robotDegree;
        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors

        /**
         * again, make sure you've changed the motor names and variables to fit your team
         */

        //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
        //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
        //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
        frontRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn));
        backRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn));
        frontLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn));
        backLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn));

            /*frontRight.setPower(driveVertical - driveHorizontal + driveTurn);
            backRight.setPower(driveVertical + driveHorizontal + driveTurn);
            frontLeft.setPower(driveVertical + driveHorizontal - driveTurn);
            backLeft.setPower(driveVertical - driveHorizontal - driveTurn);*/
    }


    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
