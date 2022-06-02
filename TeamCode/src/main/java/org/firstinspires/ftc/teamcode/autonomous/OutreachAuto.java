package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.ClawAutoThread;
import org.firstinspires.ftc.teamcode.core.Moby;
import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;

@Autonomous(name = "OutreachAuto", group = "competition")
public class OutreachAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Moby.init(hardwareMap, false);

        OdometryMecanumDrive drive = new OdometryMecanumDrive(hardwareMap);

        ClawAutoThread top;
        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0,0,0));


        ClawAutoThread ground = new ClawAutoThread(ClawAutoThread.Level.GROUND);

        // path to alliance shipping hub
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(37, -31, Math.toRadians(270))).build();

        // path to freight
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(85, 0, Math.toRadians(90))).build();

        // path to spinner
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(24, 24, Math.toRadians(-90))).build();

        top = new ClawAutoThread(ClawAutoThread.Level.TOP);
        top.start();
        drive.followTrajectory(traj1);


        Moby.intake.out();
        sleep(1500);
        Moby.intake.stopSpinner();



    }
}
