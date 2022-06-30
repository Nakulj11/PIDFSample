package org.firstinspires.ftc.teamcode.autonomous;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.teamcode.component.Arm;
        import org.firstinspires.ftc.teamcode.component.ClawAutoThread;
        import org.firstinspires.ftc.teamcode.core.Moby;
        import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;

@Autonomous(name = "Outreach Auto", group = "competition")
public class OutreachAuto extends LinearOpMode {

    ClawAutoThread arm;

    enum State {
        MOVE_ARM,
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        IDLE
    }

    final double KP = 0.02/3169;
    final double KI = (1.0/3169)/100.0;
    double integralSum=0;

    State currentState = State.IDLE;

    boolean armMove = true;


    @Override
    public void runOpMode() throws InterruptedException {


        Moby.init(hardwareMap, false);
        Moby.initIMU();



        OdometryMecanumDrive drive = new OdometryMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,0));


//        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d())
//                .addDisplacementMarker(() -> {
//                    top();
//                }).build();

        // path to shipping hub from start
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(39, -31, Math.toRadians(270)))
                .build();


        // path to freight
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(73, 13, Math.toRadians(90)))
                .build();

        // path to shipping hub from freight
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(39, -31, Math.toRadians(270)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0))).build();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;

        ElapsedTime time = new ElapsedTime();
        time.reset();
        armMove = true;
        drive.followTrajectoryAsync(traj1);

//        Moby.arm.moveArm(Arm.TurnValueAuto.TOP);
//        sleep(1000); //?

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        outtake();

                        drive.followTrajectoryAsync(traj2);
                        time.reset();
                        armMove = true;
                        Moby.intake.in();
                        currentState = State.TRAJECTORY_2;
                    }
                    if(time.milliseconds()>100&&armMove){
                        Moby.arm.moveArm(Arm.TurnValueAuto.TOP);
                        armMove=false;
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        sleep(600);
                        Moby.intake.stopSpinner();
                        Moby.arm.moveArm(Arm.TurnValueAuto.TOP);
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(traj3);

                    }
                    if(time.milliseconds()>1000&&armMove){
                        Moby.arm.moveArm(Arm.TurnValueAuto.GROUND);
                        armMove = false;
                    }
                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        outtake();

                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(traj4);
                        armMove = true;
                        time.reset();
                    }
                    break;
                case TRAJECTORY_4:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    if(time.milliseconds()>1000&&armMove){
                        Moby.arm.moveArm(Arm.TurnValueAuto.GROUND);
                        armMove = false;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            drive.update();

            if(!Moby.arm.isBusy()){
                Moby.arm.stopArm();
            }
            //            else{
//                Moby.arm.setPower(getPIControlledArmPower());
//            }
        }


    }

    public double getPIControlledArmPower(){


        double error = Moby.arm.getTargetPosition()-Moby.arm.getTicks();
        integralSum += error * Moby.arm.getTime();


        Moby.arm.reset();

        double output = error*KP + integralSum*KI;
        if(output>1){output=1;}
        if(output<-1){output=-1;}
        return output;
    }

    public void top(){
        arm = new ClawAutoThread(ClawAutoThread.Level.TOP);
        arm.start();
    }

    public void ground(){
        arm = new ClawAutoThread(ClawAutoThread.Level.GROUND);
        arm.start();
    }

    public void outtake(){
        Moby.intake.out();
        sleep(1500);
        if(Moby.colorSensor.intakeSuccessful()){
            Moby.intake.in();
            sleep(500);
            Moby.intake.out(0.7);
            sleep(1000);

        }
        Moby.intake.stopSpinner();
    }
}
