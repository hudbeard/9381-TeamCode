package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Power Play Red", group = "Android Studio")

public class HotBotsAutonRed2024 extends LinearOpMode {
    private final HotBotsRobot2024 robot   = new HotBotsRobot2024();
    private final CameraDetection camera   = new CameraDetection();
    private Integer camera_result;


    @Override
    public void runOpMode() {
        telemetry.addData("Init", "Robot...");
        telemetry.update();
        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.autonMode();
        telemetry.addData("Init", "Camera...");
        telemetry.update();
        camera.initTfod(hardwareMap, telemetry, false);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            camera_result = camera.detect();
            telemetry.addData("What I See", camera_result);
            if (camera_result == 3) {
                robot.drive(robot.val(400, 400, 400, 400), .2, true, 10);
                robot.drive(robot.val(-400, 400, 400, -400), .2, false, 10);
                robot.drive(robot.val(100, -100, -100, 100), .4, false, 10);
            } else if (camera_result == 2) {
                robot.drive(robot.val(800, 800, 800, 800), .2, true, 10);
            } else {
                robot.drive(robot.val(400, 400, 400, 400), .2, true, 10);
            }
            robot.drive(robot.val(-150, -150, -150, -150), 0.2, true, 10);
            robot.rotate(-90);
            if (camera_result == 1) {

            } else if (camera_result == 2) {
                robot.drive(robot.val(800, 800, 800, 800), 0.2, true, 10);
                robot.drive(robot.val(200, -200, -200, 200), .4, false, 10);
            } else {
                robot.drive(robot.val(650, 650, 650, 650), 0.2, true, 10);
                robot.drive(robot.val(300, -300, -300, 300), .4, false, 10);
            }
        }
    }
}
