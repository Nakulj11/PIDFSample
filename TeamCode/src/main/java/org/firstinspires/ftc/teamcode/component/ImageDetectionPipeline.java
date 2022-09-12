package org.firstinspires.ftc.teamcode.component;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ImageDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    //locations
    public enum Location {
        ONE,
        TWO,
        THREE
    }

    double[] values = new double[3];

    //location
    private Location location = Location.ONE;

    //Region of interest coordinates
    static final Rect DETECTION_ROI = new Rect(
            new Point(5, 80),
            new Point(85, 150));


    //Threshold to determine if there is an object
    final static double PERCENT_COLOR_THRESHOLD = 0.1;

    public ImageDetectionPipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);



        Scalar[] lowHSV = {new Scalar(12, 180, 130), //ONE
                new Scalar(12, 180, 130),//TWO
                new Scalar(12, 180, 130)};//THREE


        Scalar[] highHSV = {new Scalar(26, 255, 255),//ONE
                new Scalar(12, 180, 130),//TWO
                new Scalar(12, 180, 130)};//THREE

        Mat roi = null;
        //creats submatrices
        for(int i=0;i<3;i++){
            Mat mat2 = new Mat();
            Core.inRange(mat, lowHSV[i], highHSV[i], mat2);
            roi = mat2.submat(DETECTION_ROI);
            //finds the raw value that fits in HSV range
            values[i] = Core.sumElems(roi).val[0] / DETECTION_ROI.area() / 255;
        }


        roi.release();


        int highestIndex = 0;
        for(int i=1;i<3;i++){
            if(values[i]>values[highestIndex]){
                highestIndex = i;
            }
        }

        switch(highestIndex){
            case 0:
                location = Location.ONE;
                break;
            case 1:
                location = Location.TWO;
                break;
            case 2:
                location = Location.THREE;
                break;
            default:
                break;
        }

        //stuff to make the gray scale appear on on robot phone
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorRectangle = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, DETECTION_ROI, colorRectangle);

        return mat;
    }

    //some methods to get constants and vars
    public Location getLocation() {
        return location;
    }

    public double[] getValues(){
        return values;
    }

}


