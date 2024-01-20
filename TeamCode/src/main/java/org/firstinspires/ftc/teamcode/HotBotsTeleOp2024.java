package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PowerPlay TeleOp", group="Android Studio")

public class HotBotsTeleOp2024 extends LinearOpMode {
    HotBotsRobot2024 robot = new HotBotsRobot2024();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleOpMode();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double lifting_arm_pos = 10000;
        double pixel_arm_pos = 800;
        double rotate_pos = 0.75;
        double x = 0.7;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Started");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_button) {
                x = 1;
            }
            if (!gamepad1.left_stick_button) {
                x = 0.7;
            }
            if (gamepad2.a) {
                robot.claw_right.setPosition(0);
                robot.claw_left.setPosition(1);
            }
            if (gamepad2.b) {
                robot.claw_right.setPosition(.25);
                robot.claw_left.setPosition(.75);
            }
            if (gamepad2.right_bumper) {
                robot.claw_rotate.setPosition(.8);
            }
            if (gamepad2.left_bumper) {
                robot.claw_rotate.setPosition(.5);
            }
            if (gamepad2.x) {
                robot.airplane.setPosition(0.9);
            }
            if (gamepad2.y) {
                robot.airplane.setPosition(0.5);
            }
            if (gamepad1.dpad_up) {
                robot.moveLiftingArm((int) (0-lifting_arm_pos), 5);
                lifting_arm_pos = 0;
            }
            if (gamepad1.dpad_right) {
                robot.moveLiftingArm((int) (5000-lifting_arm_pos), 5);
                lifting_arm_pos = 5000;
            }
            if (gamepad1.dpad_down) {
                robot.moveLiftingArm((int) (10000-lifting_arm_pos), 5);
                lifting_arm_pos = 10000;
            }
            if (gamepad2.dpad_up) {
                robot.movePixelArm((int) (0-pixel_arm_pos), 5);
                robot.pixel_arm.setPower(-0.1);
                pixel_arm_pos = 0;
            }
            if (gamepad2.dpad_down) {
                robot.movePixelArm((int) (800-pixel_arm_pos), 5);
                robot.pixel_arm.setPower(0);
                pixel_arm_pos = 800;
            }
            if (gamepad1.right_trigger > 0.1) {
                robot.liner_actuator.setPower(1);
            }
            else if (gamepad1.left_trigger > 0.1) {
                robot.liner_actuator.setPower(-1);
            }
            if (gamepad1.right_bumper) {
                robot.lifting_arm.setPower(1);
                robot.lifting_arm_2.setPower(1);
            }
            else if (gamepad1.left_bumper) {
                robot.lifting_arm.setPower(-1);
                robot.lifting_arm_2.setPower(-1);
            }
            robot.lifting_arm.setPower(0);
            robot.lifting_arm_2.setPower(0);
            robot.liner_actuator.setPower(0);
            robot.listen_for_drive_commands(x);
            telemetry.update();
        }
    }
}
