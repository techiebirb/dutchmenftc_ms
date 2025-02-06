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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Red Magic", group = "Autonomous")

public class magic extends LinearOpMode {

    public class Shoulder {

        private DcMotorEx shoulder;
        private DcMotorEx shoulder_support;

        public Shoulder(HardwareMap hardwareMap) {
            shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
            shoulder_support = hardwareMap.get(DcMotorEx.class, "DC_Support");
            shoulder_support.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulder_support.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ShoulderUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shoulder.setPower(-0.6);
                    shoulder_support.setPower(-0.6);
                    initialized = true;
                }

                double pos = shoulder.getCurrentPosition();
                packet.put("Shoulder_Pos", pos);
                if (pos > -310) {                    /// Shoulder pos
                    return true;
                } else {
                    shoulder.setPower(0);
                    shoulder_support.setPower(0);
                    sleep(200);
                    return false;
                }
            }
        }

        public Action shoulderup() {
            return new ShoulderUp();
        }

        public class ShoulderDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shoulder.setPower(0.6);
                    shoulder_support.setPower(0.6);
                    initialized = true;
                }

                double pos = shoulder.getCurrentPosition();
                packet.put("Shoulder_Pos", pos);
                if (pos < -10) {                    /// Shoulder pos
                    return true;
                } else {
                    shoulder.setPower(0);
                    shoulder_support.setPower(0);
                    sleep(200);
                    return false;
                }
            }
        }

        public Action shoulderdown() {
            return new ShoulderDown();
        }
    } /////////////////////////////////////////////////////////////////////////////

    public class Slider{
        private DcMotorEx slider;

        public Slider(HardwareMap hardwareMap){
            slider = hardwareMap.get(DcMotorEx.class, "Slider");
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SliderUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slider.setPower(1);
                    initialized = true;
                }

                double pos = slider.getCurrentPosition();
                packet.put("Slider", pos);
                if (pos < 2100) {                        /// Slider pos
                    return true;
                } else {
                    slider.setPower(0.1);
                    return false;
                }
            }
        }

        public Action sliderup() {
            return new SliderUp();
        }

        public class SliderDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slider.setPower(-1);
                    initialized = true;
                }

                double pos = slider.getCurrentPosition();
                packet.put("Slider", pos);
                if (pos > 100) {                                  /// Slider pos
                    return true;
                } else {
                    slider.setPower(0);
                    sleep(200);
                    return false;
                }
            }
        }

        public Action sliderdown() {
            return new SliderDown();
        }
    }///////////////////////////////////////////////////////////////////

    public class Claw{
        private DcMotorEx wrist;
        private CRServo claw;

        public Claw(HardwareMap hardwareMap){
            claw = hardwareMap.get(CRServo.class, "Claw");
            wrist = hardwareMap.get(DcMotorEx.class,"Wrist");
        }

        public class WristUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }

                double pos = wrist.getCurrentPosition();
                packet.put("Slider", pos);
                if (pos < 75) {                                 /// wrist pos
                    return true;
                } else {
                    wrist.setPower(0);
                    sleep(200);
                    return false;
                }
            }
        }

        public Action wristup() {
            return new WristUp();
        }

        public class WristDw implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wrist.setPower(-1);
                    initialized = true;
                }

                double pos = wrist.getCurrentPosition();
                packet.put("Slider", pos);
                if (pos > 10) {                   /// wrist pos
                    return true;
                } else {
                    wrist.setPower(0);
                    sleep(500);
                    return false;
                }
            }
        }

        public Action wristdw() {
            return new WristDw();
        }

        public class ClawOp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPower(-0.3);              /// claw pos
                    initialized = true;
                }

                double pos = claw.getPower();
                packet.put("Slider", pos);
                if (pos > -0.3) {                   /// claw pos
                    return true;
                } else {
                    sleep(500);
                    return false;
                }
            }
        }

        public Action clawop() {
            return new ClawOp();
        }

        public class ClawCl implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPower(0.2);       /// claw pos
                    initialized = true;
                }


                double pos = claw.getPower();
                packet.put("Slider", pos);
                if (pos < 0.2) {             /// claw pos
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action clawcl() {
            return new ClawCl();
        }

    }///////////////////////////////////////////////////////////////////

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-24.5, -65, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoulder shoulder = new Shoulder(hardwareMap);
        Slider slider = new Slider(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Claw wrist = new Claw(hardwareMap);


        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(48)
                .turn(Math.toRadians(-22.5))
                .strafeTo(new Vector2d(32, 45))
                .turn(Math.toRadians(22.5))
                .strafeTo(new Vector2d(-4.5, 45))
                .turn(Math.toRadians(-22.5))
                .strafeTo(new Vector2d(5, 45))
                .turn(Math.toRadians(22.4))
                .strafeTo(new Vector2d(10, 45))
                .turn(Math.toRadians(-22.5))
                .strafeTo(new Vector2d(5, 45))
                .turn(Math.toRadians(75))
                .turn(Math.toRadians(-32.5 ))
                .strafeTo(new Vector2d(5, 45))
                .turn(Math.toRadians(22.4));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20, 30))  // 20, 61.7, 45, 55  -- 60, 60
                .turn(Math.toRadians(-20.5));
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
                .waitSeconds(0.5);
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
            trajectoryActionChosen = tab4.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        claw.clawcl(),
                        tab3.build(),
                        shoulder.shoulderup(),
                        slider.sliderup(),
                        wrist.wristup(),
                        claw.clawop(),
                        wrist.wristdw(),
                        slider.sliderdown(),
                        shoulder.shoulderdown(),
                        shoulder.shoulderup(),
                        slider.sliderup(),
                        slider.sliderdown(),
                        shoulder.shoulderdown()

                        ////////////////////////////



                )
        );
    }
}
