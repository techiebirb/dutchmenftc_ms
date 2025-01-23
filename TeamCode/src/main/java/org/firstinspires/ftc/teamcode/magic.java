package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Red Magic", group = "Autonomous")

public class magic extends LinearOpMode {



    public static class Shoulder {
        private final DcMotorEx Shoulder;

        public Shoulder(HardwareMap hardwareMap) {
            Shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
            Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ShoulderUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Shoulder.setPower(0.8);
                    initialized = true;
                }

                double pos = Shoulder.getCurrentPosition();
                packet.put("ShoulderPos", pos);
                if (pos > 92) {                  // Sholder position
                    return true;
                } else {
                    Shoulder.setPower(0);
                    return false;
                }
            }
        }

        public Action ShoulderUp(){
            return new ShoulderUp();
        }

        public class ShoulderDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Shoulder.setPower(0.8);
                    initialized = true;
                }

                double pos = Shoulder.getCurrentPosition();
                packet.put("ShoulderPos", pos);
                if (pos < 446) {                  // Sholder position
                    return true;
                } else {
                    Shoulder.setPower(0);
                    return false;
                }
            }
        }
        public Action ShoulderDown(){
            return new ShoulderDown();
        }

    }

    public static class Slider {
        private final DcMotorEx Slider;

        public Slider(HardwareMap hardwareMap) {
            Slider = hardwareMap.get(DcMotorEx.class, "Slider");
            Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slider.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SliderUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Slider.setPower(0.8);
                    initialized = true;
                }

                double pos = Slider.getCurrentPosition();
                packet.put("Slider", pos);
                if (pos < 1000) {                  // Sholder position
                    return true;
                } else {
                    Slider.setPower(0);
                    return false;
                }
            }
        }

        public Action SliderUp(){
            return new SliderUp();
        }

        public class SliderDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Slider.setPower(0.8);
                    initialized = true;
                }

                double pos = Slider.getCurrentPosition();
                packet.put("Slider", pos);
                if (pos > 10) {                  // Sholder position
                    return true;
                } else {
                    Slider.setPower(0);
                    return false;
                }
            }
        }
        public Action SliderDown(){
            return new SliderDown();
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-20.0, 61.7, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoulder Shoulder = new Shoulder(hardwareMap);
        Slider Slider = new Slider(hardwareMap);

        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(40, Math.toRadians(180))
                .strafeTo(new Vector2d(56, 40));
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

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        Shoulder.ShoulderUp(),
                        Shoulder.ShoulderDown(),

                        Slider.SliderUp(),
                        Slider.SliderDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
