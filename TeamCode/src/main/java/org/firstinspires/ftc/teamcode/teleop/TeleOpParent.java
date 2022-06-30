/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousParent;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.core.Moby;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.firstinspires.ftc.teamcode.library.DriverOrientedControl;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

/**
 * feel free to change the name or group of your class to better fit your robot
 */
public class TeleOpParent extends LinearOpMode {

    /**
     * make sure to change these motors to your team's preference and configuration
     */


    Orientation angles;
    Acceleration gravity;

    DriverOrientedControl drive;

    DriveStyle.DriveType type = DriveStyle.DriveType.MECANUMARCADE;


    public double power = 0.76;

    boolean driverOriented = true;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * you can change the variable names to make more sense
         */



        composeTelemetry();

        waitForStart();

        Moby.init(hardwareMap, true);
        if(Moby.imu==null){
            Moby.initIMU();
        }
        drive = new DriverOrientedControl();



        while (opModeIsActive()) {
            if(driverOriented){
                drive.drive2( gamepad2, power);
            }
            //switch-ability between drive types in case driverOriented malfunctions
//            else{
//                DriveStyle.driveWithType(Moby.driveMotors, gamepad2, type, POWER);
//            }
//            if(gamepad2.left_bumper){
//                driverOriented = false;
//            }
//
//            if(gamepad2.right_bumper){
//                driverOriented = true;
//            }

            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if(gamepad2.right_bumper){
                Moby.initIMU();
            }

            if (gamepad1.dpad_up|| gamepad2.dpad_up) {
                Moby.spinner.spin();
            }else if(gamepad1.dpad_down || gamepad2.dpad_down) {
                Moby.spinner.reverse();
            }else {
                Moby.spinner.stop();
            }



            if(Moby.colorSensor.intakeSuccessful()){
                Moby.light.setPower(0.15);
            }else{
                Moby.light.setPower(0);
            }



//            Moby.light.setPower(0.15 );



//
            telemetry.addData("Distance", Moby.colorSensor.getDistance());
//            telemetry.addData("Red", Moby.colorSensor.getRed());
//            telemetry.addData("Green", Moby.colorSensor.getBlue());
//            telemetry.addData("Blue", Moby.colorSensor.getGreen());
            telemetry.update();

            //to switch between slow mode, normal mode, and fast mode
            if(gamepad2.left_stick_button){
                power = 0.88;
            }

            if(gamepad2.right_stick_button){
                power = 0.15;
            }



            //garbage if statements
//            if(gamepad2.dpad_left){
//                drivetrain2.straighten(90, 0.3);
//                drivetrain2.move(DriveSensor.Sensor.RIGHT, DriveSensor.ReferenceDirection.TOWARDS, 10, 0.6);
//            }
//
//            if(gamepad2.dpad_right){
//                drivetrain2.straighten(90, 0.3);
//                drivetrain2.move(DriveSensor.Sensor.LEFT, DriveSensor.ReferenceDirection.TOWARDS, 10, 0.6);
//            }


            //changes claw position on controller input
            if(gamepad1.a||gamepad2.a){
                Moby.arm.moveArmTeleOp(Arm.TurnValueTeleOp.GROUND);
            }

            if(gamepad1.x||gamepad2.x){
                Moby.arm.moveArmTeleOp(Arm.TurnValueTeleOp.BOTTOM);
            }

            if(gamepad1.y||gamepad2.y){
                Moby.arm.moveArmTeleOp(Arm.TurnValueTeleOp.MID);
            }

            if(gamepad1.b||gamepad2.b){
                Moby.arm.moveArmTeleOp(Arm.TurnValueTeleOp.TOP);
            }


            //makes small increments or decrements to claw position
            if(gamepad1.right_trigger >= 0.1 || gamepad2.right_trigger >= 0.1) {
                Moby.arm.moveUp();
            }
            if(gamepad1.left_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
                Moby.arm.moveDown();
            }

            if(gamepad1.dpad_left||gamepad2.dpad_left) {
                Moby.intake.out();
            }else if(gamepad1.dpad_right||gamepad2.dpad_right){
                Moby.intake.in();
            }else{
                Moby.intake.stopSpinner();
            }


            if(gamepad1.left_bumper||gamepad2.left_bumper){
                Moby.intake.out(1);
            }

//            if(gamepad1.right_bumper||gamepad2.right_bumper){
//                Moby.arm.moveArmTeleOp(Arm.TurnValueTeleOp.SHARED);
//            }

//            telemetry.addData("Position", Moby.intake.getPosition());
//            telemetry.update();



            //to stop mover once it has reached its target position
            if(!Moby.arm.isBusy()){
                Moby.arm.stopArm();
            }


        }

    }

    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = Moby.imu.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = Moby.imu.getImu().getGravity();
            }
        });
    }


}