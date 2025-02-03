package org.firstinspires.ftc.teamcode;

import android.widget.ToggleButton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "little Bertha_I")
public class odo2test extends LinearOpMode {

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

        //////////////////////////////////////////////////////////////////////////////////////////////
        Pose2d initialPose = new Pose2d(-20.0, 61.7, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5);
//                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-10, 30));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-40, 30))
                .strafeTo(new Vector2d(-40, 24))
                .strafeTo(new Vector2d(-44, 24))
                .strafeTo(new Vector2d(-44, 60))
                .strafeTo(new Vector2d(-50, 24))
                .strafeTo(new Vector2d(-55, 24))
                .strafeTo(new Vector2d(-55, 60))
                .strafeTo(new Vector2d(-55, 24))
                .strafeTo(new Vector2d(-65, 24))
                .strafeTo(new Vector2d(-65, 60));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(90));


        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

////////////////////////////////////////////////////////////////////////////////////////////////////////







        waitForStart();

        while (opModeIsActive()){


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

            double Shoulder_power  = gamepad2.left_stick_y;
            double Slider_power = -gamepad2.right_stick_y;

            double wrist_power_down = -gamepad2.right_trigger;

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
                while (Shoulder.getCurrentPosition() < 0) {
                    Shoulder.setPower(0.3);
                    Shoulder_Support.setPower(0.2);
                    if(Shoulder.getCurrentPosition()> 10 && Shoulder.getCurrentPosition()<0){
                        Shoulder.setPower(0);
                        Shoulder_Support.setPower(0);
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

            if(gamepad2.b){
                while (Slider.getCurrentPosition() < 2716){
                    Slider.setPower(1);
                    if(Slider.getCurrentPosition() > 2800){
                        Slider.setPower(0);
                        break;
                    }
                }
            }

            if(top){
                while (Slider.getCurrentPosition() > 500){
                    Slider.setPower(-1);
                    if(Slider.getCurrentPosition() < 200){
                        Slider.setPower(0);
                    }
                }
                while (Shoulder.getCurrentPosition() > -420){
                    Shoulder.setPower(-0.3);
                    Shoulder_Support.setPower(-0.2);
                    if(Shoulder.getCurrentPosition()>-410 && Shoulder.getCurrentPosition()<-420){
                        Shoulder.setPower(0);
                        Shoulder_Support.setPower(0);
                        break;
                    }
                }
            }


            if(gamepad2.left_bumper){
                Shoulder_Support.setPower(Shoulder_power);
                Shoulder.setPower(Shoulder_power);
            }else{
                Shoulder_Support.setPower(Shoulder_power*0.3);
                Shoulder.setPower(Shoulder_power);
            }

            if (Slider.getCurrentPosition() < 2700) {
                Slider.setPower(Slider_power);
            }else{
                Slider.setPower(0);
            }


            boolean claw_open = gamepad2.dpad_left;
            boolean claw_close = gamepad2.dpad_right;



            Wrist.setPower(wrist_power_down);

            if (gamepad2.x){
                Wrist.setPower(-wrist_power_down);
            }


            if(gamepad2.dpad_down){
                while (Wrist.getCurrentPosition() > -10){
                    Wrist.setPower(-1);//change the values  mid = 6 low = 37 top = -68
                    if(Wrist.getCurrentPosition()>0 && Wrist.getCurrentPosition()<20){
                        Wrist.setPower(0);
                        break;
                    }
                }
            }

            if(gamepad2.right_bumper){
                while (Wrist.getCurrentPosition() < 72){
                    Wrist.setPower(1);//change the values  mid = 6 low = 37 top = -68
                    if(Wrist.getCurrentPosition()< 80 && Wrist.getCurrentPosition()>70){
                        Wrist.setPower(0);
                        break;
                    }
                }
                while (Wrist.getCurrentPosition() > 72){
                    Wrist.setPower(-1);//change the values  mid = 6 low = 37 top = -68
                    if(Wrist.getCurrentPosition()< 80 && Wrist.getCurrentPosition()> 70){
                        Wrist.setPower(0);
                        break;
                    }
                }
            }

            if(gamepad2.dpad_up){
                while (Wrist.getCurrentPosition() < 100){
                    Wrist.setPower(1);//change the values  mid = 6 low = 37 top = -68
                    if(Wrist.getCurrentPosition()>95 && Wrist.getCurrentPosition()<102){
                        Wrist.setPower(0);
                        break;
                    }
                }
            }

            if(claw_open){
                Claw.setPower(-0.6);
            }
            if(claw_close){
                Claw.setPower(-0.1);
            }


            Action trajectoryActionChosen;
            if (startPosition == 1) {
                trajectoryActionChosen = tab4.build();
            } else if (startPosition == 2) {
                trajectoryActionChosen = tab2.build();
            } else {
                trajectoryActionChosen = tab3.build();
            }

            if(gamepad2.x) {
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryActionChosen,
                                trajectoryActionCloseOut
                        )
                );
            }                        /////////////////////////////////////////




            if(gamepad2.right_bumper && gamepad2.left_bumper){
                FrontLeft.setPower(frontLeftPower);
                FrontRight.setPower(frontRightPower);
                BackLeft.setPower(backLeftPower);
                BackRight.setPower(backRightPower);
                Slider.setPower(0);
                Shoulder_Support.setPower(0);
                Shoulder.setPower(0);
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
            telemetry.addData("Steering BF", -gamepad1.left_stick_y );
            telemetry.addData("Steering LR", gamepad1.left_stick_x );
            telemetry.addData("wrist :",Wrist.getCurrentPosition());
            telemetry.update();

        }
    }
}