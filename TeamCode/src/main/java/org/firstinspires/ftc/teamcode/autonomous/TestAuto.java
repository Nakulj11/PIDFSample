package org.firstinspires.ftc.teamcode.autonomous;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;


@Autonomous(name = "TestAuto", group = "competition")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OdometryMecanumDrive drive = new OdometryMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0,0,0));

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(35, 32)).build();

        telemetry.addData("Traj done:", 1);
        telemetry.update();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToConstantHeading(new Vector2d(70, 5), Math.toRadians(0)).build();  // 0, 70

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(25).build();

        Trajectory traj4 = drive.trajectoryBuilder((traj3.end()))
                .lineToLinearHeading(new Pose2d(95, 64, Math.toRadians(180))).build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end()).forward(52).build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end()).strafeRight(14).build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end()).strafeLeft(14).build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end()).splineToLinearHeading(new Pose2d(20, 22), Math.toRadians(0)).build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end()).lineTo(new Vector2d(0,0)).build();

        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);


//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .forward(10)
//                        .build()
//        );
    }
}
