package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ITD TeleOp", group="Android Studio")

public class HotBotsTeleOp2024 extends LinearOpMode {
    HotBotsRobot2024 robot = new HotBotsRobot2024();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleOpMode();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double lifting_arm_pos = 15000;
        double pixel_arm_pos = 800;
        double rotate_pos = 0.75;
        double liner_actuator_pos = 500;
        double x = 0.7;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Started");
        telemetry.update();
        robot.slide_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_button) {
                x = .7;
            }
            //if (!gamepad1.left_stick_button) {
               // x = 0.7;
            //}
            if (gamepad1.right_stick_button)    {
                x = .1;
            }

            //if (gamepad2.right_bumper) {
            // robot.slide_3.setPosition(0.05);
            // }
            //if (gamepad2.left_bumper) {
            // robot.slide_3.setPosition(.275);
            //}
            if (gamepad2.a) {
                robot.rotate_1.setPosition(0.8);
                robot.claw_1.setPosition(.45);
            }
            if (gamepad2.b) {
                robot.claw_1.setPosition(0.6);
            }
            if (gamepad2.y) {
                robot.rotate_1.setPosition(1);
            }
            if (gamepad2.x) {
                robot.claw_2.setPosition(.5);
            }
            if (gamepad1.a) {
                robot.light_1.setPosition(1);

            }
            //if (gamepad2.dpad_up) {
            //robot.rotate_2.setPosition(0.05);
            //robot.rotate_1.setPosition(1);
            //}
            if (gamepad2.dpad_right) {
                robot.rotate_2.setPosition(.5);

            }
            //if (gamepad2.dpad_left) {
            // robot.claw_2.setPosition(1);
            //robot.rotate_1.setPosition(1);
            //}
            //if (gamepad2.dpad_down) {
            //robot.rotate_2.setPosition(.7);
            //robot.rotate_1.setPosition(1);
            //}
            if (gamepad2.right_bumper) {
                robot.slide_3.setPosition(0.275);
                sleep(1);
                robot.rotate_1.setPosition(.8);
                sleep(1);
                robot.claw_1.setPosition(.45);
                sleep(1);
            }
            if (gamepad2.left_bumper) {
                robot.rotate_1.setPosition(.475);
                sleep(1);
                robot.slide_3.setPosition(.05);
                sleep(1);
                robot.rotate_2.setPosition(.69);
                sleep(1);
                robot.claw_2.setPosition(.5);
                sleep(1000);
                robot.claw_2.setPosition(1);
                sleep(500);
                robot.claw_1.setPosition(.45);
                sleep(300);
                robot.rotate_2.setPosition(.05);
                robot.rotate_1.setPosition(.3);
            }
            if (gamepad2.dpad_up) {
                robot.slide_1.setPower(.8);
                robot.slide_2.setPower(.8);
                robot.slide_1.setTargetPosition(3000);
                robot.slide_2.setTargetPosition(3000);
                robot.slide_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slide_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (gamepad2.dpad_left) {
                robot.slide_1.setPower(.8);
                robot.slide_2.setPower(.8);
                robot.slide_1.setTargetPosition(2000);
                robot.slide_2.setTargetPosition(2000);
                robot.slide_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slide_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (gamepad2.dpad_down) {
                robot.slide_1.setPower(.8);
                robot.slide_2.setPower(.8);
                robot.slide_1.setTargetPosition(0);
                robot.slide_2.setTargetPosition(0);
                robot.slide_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slide_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            //if (gamepad2.dpad_up) {
            // robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.slide.setTargetPosition(100);
            //robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.slide.setPower(.1);

            //if (gamepad1.dpad_up) {
            //robot.moveLiftingArm((int) (0-lifting_arm_pos), 5);
            //lifting_arm_pos = 0;

            //if (gamepad1.dpad_right) {
            //robot.moveLiftingArm((int) (7000-lifting_arm_pos), 5);
            //lifting_arm_pos = 7000;

            //if (gamepad1.dpad_down) {
            // robot.moveLiftingArm((int) (15000-lifting_arm_pos), 5);
            //lifting_arm_pos = 15000;

            //if (gamepad1.y) {
            // robot.moveLinearActuator((int) (500-liner_actuator_pos), 5);
            //liner_actuator_pos = 500;

            //if (gamepad1.b) {
            //robot.moveLinearActuator((int) (300 - liner_actuator_pos), 5);
            //liner_actuator_pos = 300;

            //if (gamepad1.a) {
            //robot.moveLinearActuator((int) (0 - liner_actuator_pos), 5);
            //liner_actuator_pos = 0;


            //if (gamepad1.right_trigger > 0.1) {
            // robot.liner_actuator.setPower(1);
            // } else if (gamepad1.left_trigger > 0.1) {
            //robot.liner_actuator.setPower(-1);
            // }
            // if (gamepad1.right_bumper) {
            // robot.hook_1.setPower(1);
            //robot.lifting_arm_2.setPower(1);
            // } else if (gamepad1.left_bumper) {
            // robot.hook_1.setPower(-1);
            //robot.lifting_arm_2.setPower(-1);
            // }
            // robot.hook_1.setPower(0);
            //robot.lifting_arm_2.setPower(0);
            //robot.liner_actuator.setPower(0);
            // robot.slide_1.setPower(gamepad2.right_stick_y +.1);


            robot.listen_for_drive_commands(x);
            telemetry.update();
        }
    }
}

