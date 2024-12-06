package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Power Play Blue Far", group = "Android Studio")

public class Far_Side_Blue extends LinearOpMode {
        private final HotBotsRobot2024 robot   = new HotBotsRobot2024();
        private final CameraDetection camera   = new CameraDetection();
        private Integer camera_result;
    @Override
    public void runOpMode() {
        telemetry.addData("Init", "Robot...");
        telemetry.update();
        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        camera.initTfod(hardwareMap, telemetry, false);
        robot.autonMode();
        telemetry.addData("Init", "Camera...");
        telemetry.update();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            camera_result = camera.detect();
            telemetry.addData("What I See", camera_result);

            //robot.claw_right.setPosition(0);
            //robot.claw_left.setPosition(1);
            sleep(500);

            if (camera_result == 3) { //right
                robot.drive(robot.val(450, 450, 450, 450), .2, true, 10);
                robot.rotate(-60);
                robot.drive(robot.val(150, 150, 150, 150), .2, true, 10);
                robot.drive(robot.val(-200, -200, -200, -200), .2, true, 10);
                robot.rotate(0);
                robot.drive(robot.val(-50, 50, 50, -50), .1, false, 10);
                robot.drive(robot.val(-350, -350, -350, -350), .2, true, 10);


            } else if (camera_result == 2) {
                robot.drive(robot.val(800, 800, 800, 800), .2, true, 10);
            } else {//camera_result==1 left

                robot.drive(robot.val(450, 450, 450, 450), .2, true, 10);
                robot.rotate(60);
                robot.drive(robot.val(150, 150, 150, 150), .2, true, 10);
                robot.drive(robot.val(-260, -260, -260, -260), .2, true, 10);
            }


            robot.drive(robot.val(-150, -150, -150, -150), 0.2, true, 10);
            robot.rotate(90);
            sleep(8000);


            if (camera_result == 1) {
                robot.drive(robot.val(350, -350, -350, 350), .1, false, 10);
                robot.drive(robot.val(2300, 2300, 2300, 2300), .3, true, 10);
                robot.rotate(90);
                robot.drive(robot.val(-400, 400, 400, -400), .3, false, 10);
                robot.drive(robot.val(100, 100, 100, 100), .1, true, 10);
                //robot.claw_right.setPosition(.25);
                //robot.claw_left.setPosition(.75);
                robot.drive(robot.val(-200, -200, -200, -200), .1, true, 10);
                //robot.drive(robot.val(-600, 600, 600, -600), 0.2, true, 10);
            } else if (camera_result == 2) {
                robot.drive(robot.val(-100, 100, 100, -100), .1, false, 10);
                robot.drive(robot.val(2000, 2000, 2000, 2000), 0.3, true, 10);
                robot.drive(robot.val(100, 100, 100, 100), 0.1, true, 10);
                robot.drive(robot.val(-100, 100, 100, -100), .1, false, 10);
                //robot.claw_right.setPosition(.25);
                //robot.claw_left.setPosition(.75);
                robot.drive(robot.val(-200, -200, -200, -200), .1, true, 10);
                //robot.drive(robot.val(-600, 600, 600, -600), 0.2, true, 10);
            } else { //camera result==3
                robot.drive(robot.val(1900, 1900, 1900, 1900), 0.3, true, 10);
                robot.drive(robot.val(-775, 775, 775, -775), .2, false, 10);
                robot.rotate(90);
                robot.drive(robot.val(150, 150, 150, 150), 0.1, true, 10);
                //robot.claw_right.setPosition(.25);
                //robot.claw_left.setPosition(.75);
                sleep(1000);
                robot.drive(robot.val(-150, -150, -150, -150), .1, true, 10);
                //robot.drive(robot.val(-600, 600, 600, -600), 0.2, true, 10);
            }
        }
    }
}
