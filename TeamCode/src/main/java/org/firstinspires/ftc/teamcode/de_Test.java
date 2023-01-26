package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="de_test", group="EXP")
public class de_Test extends LinearOpMode {

    public String a;
    OpenCvCamera camera;
    private ConeDetector cd = new ConeDetector(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        startCam();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("FPS", camera.getFps());
            telemetry.addData("COLOR", cd.getLocation());
            telemetry.addData("Yellow %", cd.a);

            switch (cd.getLocation()) {
                case BLUE:
                    a = "blue";
                    break;
                case YELLOW:
                    a = "yellow";
                    break;
                case MIX:
                    a = "MIx";
                    break;
            }
            telemetry.addData("Color: ", a);
            telemetry.update();
        }
    }

    private void startCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(cd);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
}
