package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "AutoRight", group = "EXP")

//DIRECTION L1 L2 R1 R2
// - - - - FORWARD
// - + + - RIGHT
// + - - + LEFT
// + + + + BACK

//@Disabled
public class AutoRightAngel extends LinearOpMode
{

    OpenCvCamera camera;
    private ConeDetector cd = new ConeDetector(telemetry);
    private int place = 1;

    private DcMotor left_1;
    private DcMotor left_2;
    private DcMotor right_1;
    private DcMotor right_2;
    private DcMotor turner;

    private Servo claw, claw_upper1, claw_upper2;
    private Servo claw_joint;

    private final double sp = 0.25;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //колеса
        left_1 = hardwareMap.get(DcMotor.class, "left_1");
        left_2 = hardwareMap.get(DcMotor.class, "left_2");
        right_1 = hardwareMap.get(DcMotor.class, "right_1");
        right_2 = hardwareMap.get(DcMotor.class, "right_2");
        turner = hardwareMap.get(DcMotor.class, "turner");

        //клешня
        claw = hardwareMap.get(Servo.class, "claw");
        claw_joint = hardwareMap.get(Servo.class, "claw_turn");
        claw_upper1 = hardwareMap.get(Servo.class, "upper1");
        claw_upper2 = hardwareMap.get(Servo.class, "upper2");
        claw_upper2.setDirection(Servo.Direction.REVERSE);

        setDirection(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroEncoder();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startCam();

        waitForStart();

        switch (cd.getLocation()) {
            case BLUE:
                place = 1;
                break;
            case YELLOW:
                place = 2;
                break;
            case MIX:
                place = 3;
                break;
        }

        claw.setPosition(0.4);

        sleep(1000);

        claw_joint.setPosition(0.0);
        setUpperPos(0.01);

        sleep(1000);

        claw_joint.setPosition(0.0);
        setUpperPos(0.01);

        sleep(1000);

        claw.setPosition(0.32);

        while (opModeIsActive()) {

            while (!encoderMove(100) & opModeIsActive()) {
                setPower(-sp, -sp, -sp, -sp);
                telemetry.addData("ENCODER", left_1.getCurrentPosition());
                telemetry.addData("0", "0");
                telemetry.update();
            }

            setPower(0, 0, 0, 0);
            setZeroEncoder();

            while (!encoderMove(1100) & opModeIsActive()) {
                setPower(sp, -sp, -sp, sp);
                telemetry.addData("ENCODER", left_1.getCurrentPosition());
                telemetry.addData("1", "1");
                telemetry.update();
            }

            setPower(0, 0, 0, 0);
            setZeroEncoder();

            while (!encoderMove(2000) & opModeIsActive()) {
                setPower(-sp, -sp, -sp, -sp);
                telemetry.addData("ENCODER", left_1.getCurrentPosition());
                telemetry.addData("2", "2");
                telemetry.update();
            }

            setPower(0, 0, 0, 0);
            setZeroEncoder();

            while (!encoderMove(1000) & opModeIsActive()) {
                setPower(-sp, sp, sp, -sp);
                telemetry.addData("ENCODER", left_1.getCurrentPosition());
                telemetry.addData("3", "3");
                telemetry.update();
            }

            setPower(0, 0, 0, 0);
            setZeroEncoder();

            while (!encoderMove(300) & opModeIsActive()) {
                setPower(sp, sp, -sp, -sp);
                telemetry.addData("ENCODER", left_1.getCurrentPosition());
                telemetry.addData("4", "4");
                telemetry.update();
            }

            setPower(0, 0, 0, 0);
            setZeroEncoder();

            while (!encoderMove(600) & opModeIsActive()) {
                setPower(-sp, -sp, -sp, -sp);
                telemetry.addData("ENCODER", left_1.getCurrentPosition());
                telemetry.addData("5", "5");
                telemetry.update();
            }

            setPower(0, 0, 0, 0);
            setZeroEncoder();

            break;
        }
        claw_joint.setPosition(1.0);
        sleep(1000);
        claw.setPosition(0.4);

        sleep(1000);

        claw_joint.setPosition(1.0);
        setUpperPos(0.45);

        sleep(1000);

        claw_joint.setPosition(0.0);
        setUpperPos(0.1);

        if(place == 1) {
            while(opModeIsActive()) {

                while (!encoderMove(600) & opModeIsActive()) {
                    setPower(sp, sp, sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("6", "6");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                while (!encoderMove(300) & opModeIsActive()) {
                    setPower(-sp, -sp, sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("7", "7");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();
                setPower(0, 0, 0, 0);
                setZeroEncoder();

                while (!encoderMove(1000) & opModeIsActive()) {
                    setPower(-sp, sp, sp, -sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("9", "9");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                break;
            }
        }
        else if(place == 2) {
            while(opModeIsActive()) {

                while (!encoderMove(600) & opModeIsActive()) {
                    setPower(sp, sp, sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("6", "6");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                while (!encoderMove(300) & opModeIsActive()) {
                    setPower(-sp, -sp, sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("7", "7");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                while (!encoderMove(100) & opModeIsActive()) {
                    setPower(-sp, sp, sp, -sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("9", "9");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                break;
            }
        }
        else if(place == 3) {
            while(opModeIsActive()) {

                while (!encoderMove(600) & opModeIsActive()) {
                    setPower(sp, sp, sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("6", "6");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                while (!encoderMove(300) & opModeIsActive()) {
                    setPower(-sp, -sp, sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("7", "7");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();


                setPower(0, 0, 0, 0);
                setZeroEncoder();

                while (!encoderMove(1000) & opModeIsActive()) {
                    setPower(sp, -sp, -sp, sp);
                    telemetry.addData("ENCODER", left_1.getCurrentPosition());
                    telemetry.addData("9", "9");
                    telemetry.update();
                }

                setPower(0, 0, 0, 0);
                setZeroEncoder();

                break;
            }
        }
    }

    private void setZeroEncoder() {
        left_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left_1.setZeroPowerBehavior(behavior);
        left_2.setZeroPowerBehavior(behavior);
        right_1.setZeroPowerBehavior(behavior);
        right_2.setZeroPowerBehavior(behavior);
    }

    private void setMode(DcMotor.RunMode mode) {
        left_1.setMode(mode);
        left_2.setMode(mode);
        right_1.setMode(mode);
        right_2.setMode(mode);
    }

    private void setDirection(DcMotorSimple.Direction... direction) {
        left_1.setDirection(direction[0]);
        left_2.setDirection(direction[0]);
        right_1.setDirection(direction[1]);
        right_2.setDirection(direction[1]);
    }

    public void setPower(double... power) {
        left_1.setPower(power[0]);
        left_2.setPower(power[1]);
        right_1.setPower(power[2]);
        right_2.setPower(power[3]);
    }

    private boolean encoderMove(double encoder) {
        if(Math.abs(left_1.getCurrentPosition()) >= encoder & (Math.abs(right_2.getCurrentPosition()) >= encoder))
            return true;
        else
            return false;
    }

    private void setUpperPos(double pos) {
        claw_upper2.setPosition(pos);
        claw_upper1.setPosition(pos);
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
