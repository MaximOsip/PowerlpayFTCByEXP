package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.nio.charset.CoderMalfunctionError;

@TeleOp(name="TelePopka", group="EXP")

public class TelePopka extends LinearOpMode {

    private DcMotor left_1;
    private DcMotor left_2;
    private DcMotor right_1;
    private DcMotor right_2;
    private DcMotor turner;

    private Servo claw, claw_upper1, claw_upper2;
    private Servo claw_joint;

    final float management_speed = 0.001f;

    private double x, y, turn;
    private double claw_pos = 0.75, upper_position_1 = 1, upper_position_2 = 1, claw_joint_pos = 0.45;
    private double left_1_spd, left_2_spd, right_1_spd, right_2_spd;

    private ConeDetector cd = new ConeDetector(telemetry);

    public void initRobot() {

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
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initRobot();

        waitForStart();

        while (!isStopRequested() & opModeIsActive())
        {
            //region wheels
            x = -gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
//-
            right_2_spd = Range.clip(y - x , -0.6, 0.6);
            right_1_spd = Range.clip(y + x, -0.6, 0.6);
   //+
            left_2_spd = Range.clip(y + x, -0.6, 0.6);
            left_1_spd = Range.clip(y - x, -0.6, 0.6);

            setPower(-left_1_spd, -left_2_spd, -right_1_spd, -right_2_spd);

            telemetry.addData("DIRECTION_L1", left_1_spd);
            telemetry.addData("DIRECTION_L2", left_2_spd);
            telemetry.addData("DIRECTION_R1", right_1_spd);
            telemetry.addData("DIRECTION_R2", right_2_spd);

            //endregion


            //region dpads
            if(gamepad1.dpad_left) {
                claw.setPosition(0.32);
                claw_pos=0.32;
                safeSleep(400);
                claw_joint_pos = 0.0;
                upper_position_1 = 0.3;
                upper_position_2 = 0.3;
            }
            else if(gamepad1.dpad_down) {
                claw.setPosition(0.32);
                claw_pos=0.32;
                safeSleep(400);
                claw_joint_pos = 0.0;
                upper_position_1 = 0.55;
                upper_position_2 = 0.55;
            }
            else if(gamepad1.dpad_up) {
                claw.setPosition(0.32);
                claw_pos=0.32;
                safeSleep(400);
                claw_joint_pos = 0.0;
                upper_position_1 = 0.1;
                upper_position_2 = 0.1;
            }else if(gamepad1.dpad_right)
            {
                claw_joint_pos = 0.377;
                claw_pos = 0.1;
                claw.setPosition(claw_pos);
                claw_joint.setPosition(claw_joint_pos);

                while(upper_position_1 < 0.84 && opModeIsActive())
                {
                    upper_position_1 += 0.01;
                    upper_position_2 += 0.01;
                    claw_upper1.setPosition(upper_position_1);
                    claw_upper2.setPosition(upper_position_2);
                    safeSleep(20);
                }
            }
            //endregion


            //region bumpers
            if(gamepad1.right_bumper)
            {
                turner.setDirection(DcMotorSimple.Direction.FORWARD);
                turner.setPower(0.35);
            }
            else if(gamepad1.left_bumper)
            {
                turner.setDirection(DcMotorSimple.Direction.REVERSE);
                turner.setPower(0.35);
            }
            else{turner.setPower(0.0);}
            upper_position_1 += gamepad1.left_trigger*0.01;
            upper_position_2 += gamepad1.left_trigger*0.01;

            upper_position_1 -= gamepad1.right_trigger*0.005;
            upper_position_2 -= gamepad1.right_trigger*0.005;

            upper_position_1 = Range.clip(upper_position_1, 0.1, 0.84);
            upper_position_2 = Range.clip(upper_position_2, 0.1, 0.84);

            claw_upper1.setPosition(upper_position_1);
            claw_upper2.setPosition(upper_position_2);

            telemetry.addData("upper1", upper_position_1);
            telemetry.addData("upper2", upper_position_2);

            //endregion


            //region claw
            if(gamepad1.a)
            {
                claw_pos += management_speed;
            }
            else if(gamepad1.b)
            {
                claw_pos -= management_speed;
            }
            claw_pos = Range.clip(claw_pos, 0.01, 0.32);
            claw.setPosition(claw_pos);
            telemetry.addData("CLAW", claw_pos);
            telemetry.addData("CLAW_POS",  claw.getPosition());
            //endregion


            //region claw_angle
            if(gamepad1.x)
            {
                claw_joint_pos += management_speed;
            }
            else if(gamepad1.y)
            {
                claw_joint_pos -= management_speed;
            }
            claw_joint_pos = Range.clip(claw_joint_pos, 0.0, 1.0);
            claw_joint.setPosition(claw_joint_pos);

            telemetry.addData("CLAW_ANGLE", claw_joint_pos);
            telemetry.addData("ANGLe_POS", claw_joint.getPosition());
            //endregion nikita horoshiy

            telemetry.update();
        }
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

    public void safeSleep(long millis){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(!isStopRequested() && opModeIsActive() && timer.milliseconds()<=millis){}
    }
}
