package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.core.Moby;
import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;


public class  AutonomousParent extends LinearOpMode {



    //constants for power
    final double POWER = 0.7;
    final double STRAIGHTEN_POWER = 0.3;
    final double LOW_POWER = 0.5;
    double distance = 0;
    double seconds = 0;
    OdometryMecanumDrive drive = new OdometryMecanumDrive(hardwareMap);


    @Override
    public void runOpMode() throws InterruptedException {

        // Send diagnostics to user
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        Moby.init(hardwareMap, false);


        // Send diagnostics to user
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        //telemetry of the vision data
        while (!isStarted() && !isStopRequested()) {


            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), Math.toRadians(90))
                    .build();

            drive.followTrajectory(traj);

            Trajectory traj2 = drive.trajectoryBuilder(traj.end()).forward(10).build();

            drive.followTrajectory(traj2);


        }











    }




}


