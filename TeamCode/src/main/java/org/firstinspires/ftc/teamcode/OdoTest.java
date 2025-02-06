package org.firstinspires.ftc.teamcode;

import android.widget.ToggleButton;

import com.qualcomm.robotcore.eventloop.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "little Bertha")
public class OdoTest extends LinearOpMode {

    double integralSum = 0;
    double Kd = 0;
    double Ki = 0;
    double Kp = 0;

    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor BackRight = hardwareMap.get(DcMotor.class, "rightBack");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// Initiating DC motors for drivetrain complete
        DcMotorEx Shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
        DcMotor Shoulder_Support = hardwareMap.get(DcMotor.class,"DC_Support");
        DcMotor Slider = hardwareMap.get(DcMotor.class,"Slider");
        DcMotor Wrist = hardwareMap.get(DcMotor.class,"Wrist");

        Wrist.setDirection(DcMotorSimple.Direction.FORWARD);

        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoulder_Support.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shoulder_Support.setDirection(DcMotorSimple.Direction.REVERSE);
        Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shoulder_Support.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shoulder_Support.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ///Shoulder Setup Finished

        CRServo Claw = hardwareMap.get(CRServo.class,"Claw");






        waitForStart();


        while (opModeIsActive()) {


            /// Gamepad 1
            /// Driving Meccunam
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.7; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            FrontLeft.setPower(frontLeftPower);
            FrontRight.setPower(frontRightPower);
            BackLeft.setPower(-(backLeftPower));
            BackRight.setPower(-(backRightPower));

            /// Gamepad 2

            double Shoulder_power = gamepad2.left_stick_y;
            double Slider_power = -gamepad2.right_stick_y;

            double wrist_power_down = gamepad2.right_trigger;


            boolean low = gamepad2.a;
            boolean mid = gamepad2.b;
            boolean top = gamepad2.y;


            if (low) {
                while (Slider.getCurrentPosition() > 500) {
                    Slider.setPower(-1);
                    if (Slider.getCurrentPosition() < 200) {
                        Slider.setPower(0);
                    }
                }
                while (Shoulder.getCurrentPosition() < 0) {
                    Shoulder.setPower(0.3);
                    Shoulder_Support.setPower(0.2);
                    if (Shoulder.getCurrentPosition() > 10 && Shoulder.getCurrentPosition() < 0) {
                        Shoulder.setPower(0);
                        Shoulder_Support.setPower(0);
                        break;
                    }
                }
            }

            if (gamepad1.right_bumper) {
                FrontLeft.setPower(frontLeftPower * 0.3);
                FrontRight.setPower(frontRightPower * 0.3);
                BackLeft.setPower((backLeftPower * 0.3));
                BackRight.setPower((backRightPower * 0.3));
            }

            if (gamepad2.b) {
                while (Slider.getCurrentPosition() < 2716) {
                    Slider.setPower(1);
                    if (Slider.getCurrentPosition() > 2800) {
                        Slider.setPower(0);
                        break;
                    }
                }
            }

            if (top) {
                while (Slider.getCurrentPosition() > 500) {
                    Slider.setPower(-1);
                    if (Slider.getCurrentPosition() < 200) {
                        Slider.setPower(0);
                    }
                }
                while (Shoulder.getCurrentPosition() > -420) {
                    Shoulder.setPower(-0.3);
                    Shoulder_Support.setPower(-0.2);
                    if (Shoulder.getCurrentPosition() > -410 && Shoulder.getCurrentPosition() < -420) {
                        Shoulder.setPower(0);
                        Shoulder_Support.setPower(0);
                        break;
                    }
                }
            }

            if (gamepad2.left_bumper) {
                Shoulder_Support.setPower(Shoulder_power);
                Shoulder.setPower(Shoulder_power);
            } else {
                Shoulder_Support.setPower(Shoulder_power * 0.3);
                Shoulder.setPower(Shoulder_power);
            }

            Slider.setPower(Slider_power);

            if (Shoulder.getCurrentPosition() > -100){
                if (Slider.getCurrentPosition() > 1400) {
                    Slider.setPower(0);
                    gamepad2.rumble(500);
                }
            }

            boolean claw_open = gamepad2.dpad_left;
            boolean claw_close = gamepad2.dpad_right;

            Wrist.setPower(wrist_power_down);

            if (gamepad2.dpad_down){
                Wrist.setPower(-wrist_power_down);
            }


//            if(gamepad2.dpad_down){
//                while (Wrist.getCurrentPosition() > -10){
//                    Wrist.setPower(-1);//change the values  mid = 6 low = 37 top = -68
//                    if(Wrist.getCurrentPosition()>0 && Wrist.getCurrentPosition()<20){
//                        Wrist.setPower(0);
//                        break;
//                    }
//                }
//            }

//            if(gamepad2.right_bumper){
//                while (Wrist.getCurrentPosition() < 72){
//                    Wrist.setPower(1);//change the values  mid = 6 low = 37 top = -68
//                    if(Wrist.getCurrentPosition()< 80 && Wrist.getCurrentPosition()>70){
//                        Wrist.setPower(0);
//                        break;
//                    }
//                }
//                while (Wrist.getCurrentPosition() > 72){
//                    Wrist.setPower(-1);//change the values  mid = 6 low = 37 top = -68
//                    if(Wrist.getCurrentPosition()< 80 && Wrist.getCurrentPosition()> 70){
//                        Wrist.setPower(0);
//                        break;
//                    }
//                }
//            }
//
//            if(gamepad2.dpad_up){
//                while (Wrist.getCurrentPosition() < 100){
//                    Wrist.setPower(1);//change the values  mid = 6 low = 37 top = -68
//                    if(Wrist.getCurrentPosition()>95 && Wrist.getCurrentPosition()<102){
//                        Wrist.setPower(0);
//                        break;
//                    }
//                }
//            }

            if(claw_open){
                Claw.setPower(-0.3);
            }
            if(claw_close){
                Claw.setPower(0.2);
            }



            telemetry.addData("Shoulder", Shoulder.getCurrentPosition());
            telemetry.addData("Shoulder_Support", Shoulder_Support.getCurrentPosition());
            telemetry.addData("Slider", Slider.getCurrentPosition());
            telemetry.addData("Steering BF", -gamepad1.left_stick_y );
            telemetry.addData("Steering LR", gamepad1.left_stick_x );
            telemetry.addData("wrist :",Wrist.getCurrentPosition());
            telemetry.update();

        }
    }
}