package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetector extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();

    String a;

    double value;

    public enum Location {
        BLUE,
        YELLOW,
        MIX
    }
    private static Location location = Location.MIX;

    public double GetValue()
    {
        return  value;
    }

    public ConeDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        final Rect r = new Rect(
                new Point(input.cols()*225f/530f, input.rows()*230f/530f),
                new Point(input.cols()*115f/330f, input.rows()*80f/330f));

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(mat, mat, 2);

        Imgproc.threshold(mat, mat, 125, 255, Imgproc.THRESH_BINARY_INV);

        Mat mr1 = mat.submat(r);


        value = Core.sumElems(mr1).val[0] / r.area() / 255;

        mr1.release();

        Scalar white = new Scalar(255, 255, 255);

        Imgproc.rectangle(mat, r, white, 2);

        if (Math.round(value * 100) > 95) {
            location = Location.YELLOW;
        }
        else if (Math.round(value * 100) > 5) {
            location = Location.MIX;
        }
        else {
            location = Location.BLUE;
        }

        a = Math.round(value * 100) + "%";

        return mat;
    }

    public static Location getLocation() {
        return (Location) location;
    }
}
