package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.component.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class CVTest extends LinearOpMode {
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 822.317;
    double fy = 822.317f;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    Location location = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");

                telemetry.update();
            }
        });

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Location.ONE.getId()) {
                        location = Location.ONE;
                        tagOfInterest = tag;
                        break;
                    } else if (tag.id == Location.TWO.getId()) {
                        location = Location.TWO;
                        tagOfInterest = tag;
                        break;
                    } else if (tag.id == Location.THREE.getId()) {
                        location = Location.THREE;
                        tagOfInterest = tag;
                        break;
                    }
                }

            }


            if (location != null && tagOfInterest != null){
                telemetry.addData("Location", location);
                telemetry.addLine("\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
        }
            webcam.stopStreaming();




    }










    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public enum Location {
        ONE (0),
        TWO (1),
        THREE (2);

        int id;

        Location(int id){
            this.id = id;
        }

        int getId(){
            return id;
        }
    }
}
