package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "A1 Auto")

public class Auto extends LinearOpMode {
   DcMotor FrontLeft;
   DcMotor FrontRight;
   DcMotor BackLeft;
   DcMotor BackRight;
    DcMotor Slider;DcMotorEx Shoulder;
    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        BackRight = hardwareMap.get(DcMotor.class, "backRight");
         Shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");



        waitForStart();

        if(opModeIsActive()){
            moveForward(1);
            sleep(1000);
            stopMotors();
            slideLeft(1);
            sleep(1000);
            stopMotors();
            moveBackward(1);
            sleep(1000);
            stopMotors();

        }

    }
    private void moveForward(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        BackRight.setPower(-power);
    }
    private void slideLeft(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);
    }

    private void slideRight(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    private void moveBackward(double power){
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }

    // Method to stop all motors
    private void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
}


