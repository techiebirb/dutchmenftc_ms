package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "little Bertha")
public class OdoTest extends LinearOpMode {

    double integralSum = 0;
    double Kd = 0;
    double Ki = 0;
    double Kp = 0;

    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor BackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor BackRight = hardwareMap.get(DcMotor.class, "backRight");

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
        DcMotor Spool = hardwareMap.get(DcMotor.class,"Spool");

        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoulder_Support.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shoulder_Support.setDirection(DcMotorSimple.Direction.REVERSE);

        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shoulder_Support.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ///Shoulder Setup Finished

        CRServo Claw = hardwareMap.get(CRServo.class,"Claw");
        CRServo wrist1 = hardwareMap.get(CRServo.class,"wrist1");
        CRServo wrist2 = hardwareMap.get(CRServo.class,"wrist2");




        waitForStart();

        while (opModeIsActive()){


            /// Gamepad 1
            /// Driving Meccunam
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.7; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

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

            double Shoulder_power  = gamepad2.left_stick_y;
            double Slider_power = -gamepad2.right_stick_y;
            boolean RB = gamepad2.right_bumper;
            boolean Sp = gamepad2.x;
            boolean LB = gamepad2.left_bumper;

            boolean low = gamepad2.a;
            boolean mid = gamepad2.b;
            boolean top = gamepad2.y;


            if(low){
                while (Slider.getCurrentPosition() > 500){
                    Slider.setPower(-1);
                    if(Slider.getCurrentPosition() < 200){
                        Slider.setPower(0);
                    }
                }
                while (Shoulder.getCurrentPosition() < -40) {
                    Shoulder.setPower(0.1);
                    Shoulder_Support.setPower(0.2);
                    Spool.setPower(0.4);
                    if(Shoulder.getCurrentPosition()> -50 && Shoulder.getCurrentPosition()<50){
                        Shoulder.setPower(0);
                        Shoulder_Support.setPower(0);
                        Spool.setPower(0);
                        break;
                    }
                }
            }

            if(gamepad1.right_bumper){
                FrontLeft.setPower(frontLeftPower*0.3);
                FrontRight.setPower(frontRightPower*0.3);
                BackLeft.setPower((backLeftPower*0.3));
                BackRight.setPower((backRightPower*0.3));
            }

            if(top){
                while (Slider.getCurrentPosition() > 500){
                    Slider.setPower(-1);
                    if(Slider.getCurrentPosition() < 200){
                        Slider.setPower(0);
                    }
                }
                while (Shoulder.getCurrentPosition() > -340){
                    Shoulder.setPower(-0.1);
                    Shoulder_Support.setPower(-0.2);
                    Spool.setPower(-0.5);
                    if(Shoulder.getCurrentPosition()>-330 && Shoulder.getCurrentPosition()<-300){
                        Shoulder.setPower(0);
                        Shoulder_Support.setPower(0);
                        Spool.setPower(0);
                        break;
                    }
                }

            }

            while(mid){
                Slider.setPower(1);
                if (Slider.getCurrentPosition() > 740){
                    Slider.setPower(0);
                    mid = false;
                }
            }

            if(gamepad2.left_bumper){
                Shoulder_Support.setPower(Shoulder_power);
                Shoulder.setPower(Shoulder_power);
            }else{
                Shoulder_Support.setPower(Shoulder_power*0.3);
                Shoulder.setPower(Shoulder_power);
            }


            Slider.setPower(Slider_power);

            if (Slider.getCurrentPosition() > 7700){
                Slider.setPower(0);
                gamepad2.rumble(500);
            }

            boolean claw_open = gamepad2.dpad_left;
            boolean claw_close = gamepad2.dpad_right;
            boolean claw_up = gamepad2.dpad_up;
            boolean claw_down = gamepad2.dpad_down;




            if(claw_open){
                Claw.setPower(-1);
            }
            if(claw_close){
                Claw.setPower(0);
            }


            if(claw_up){
//                wrist1.setPower(0.5);
                wrist2.setPower(1);
            }
            if(claw_down){
//                wrist1.setPower(1);
                wrist2.setPower(0.1);
            }
            if(gamepad2.right_bumper){
                wrist2.setPower(0.5);
            }


            if(gamepad2.right_bumper && gamepad2.left_bumper){
                FrontLeft.setPower(frontLeftPower);
                FrontRight.setPower(frontRightPower);
                BackLeft.setPower(backLeftPower);
                BackRight.setPower(backRightPower);
                Slider.setPower(0);
                Shoulder_Support.setPower(0);
                Shoulder.setPower(0);
                Spool.setPower(0);
                while (Slider.getCurrentPosition() > 500){
                    Slider.setPower(-1);
                    if(Slider.getCurrentPosition() < 200){
                        Slider.setPower(0);
                    }
                }

            }

            telemetry.addData("Shoulder", Shoulder.getCurrentPosition());
            telemetry.addData("Shoulder_Support", Shoulder_Support.getCurrentPosition());
            telemetry.addData("Slider", Slider.getCurrentPosition());
            telemetry.addData("Wrist1 Power", wrist1.getPower());
            telemetry.addData("Wrist2 Power", wrist2.getPower());
            telemetry.addData("Steering BF", -gamepad1.left_stick_y );
            telemetry.addData("Steering LR", gamepad1.left_stick_x );
            telemetry.update();

        }
    }
}