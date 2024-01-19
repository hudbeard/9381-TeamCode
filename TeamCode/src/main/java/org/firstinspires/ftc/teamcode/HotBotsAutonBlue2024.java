package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Power Play Blue", group = "Android Studio")

public class HotBotsAutonBlue2024 extends LinearOpMode {
    private final HotBotsRobot2024 robot   = new HotBotsRobot2024();
    private final Gyro             gyro    = new Gyro();
    private final CameraDetection camera   = new CameraDetection();


    @Override
    public void runOpMode() {
        telemetry.addData("Init", "Robot...");
        telemetry.update();
        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        camera.initTfod(hardwareMap, telemetry, false);
        robot.autonMode();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Pos", camera.detect());
            telemetry.update();
            sleep(5000);
        }
    }
}
