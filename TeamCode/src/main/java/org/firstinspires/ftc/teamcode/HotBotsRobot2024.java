package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class HotBotsRobot2024 {
    public DcMotorEx front_right;
    public DcMotorEx front_left;
    public DcMotorEx back_right;
    public DcMotorEx back_left;
    public DcMotor slide_1;
    public DcMotor hook_1;
    public DcMotor hook_2;
    public DcMotor slide_2;
    public Servo slide_3;
    public Servo claw_2;
    public Servo rotate_2;
    public Servo rotate_1;
    public Servo claw_1;
    public Servo light_1;
    HardwareMap hwMap;
    Telemetry telemetry;
    Gamepad GamePad1;
    Gamepad GamePad2;
    public final Gyro gyro             = new Gyro();
    public final ElapsedTime runtime   = new ElapsedTime();
    public double target               = 0;

    public HotBotsRobot2024(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry t, Gamepad g1, Gamepad g2) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        GamePad1 = g1;
        GamePad2 = g2;
        telemetry = t;

        gyro.init_gyro(hwMap);

        runtime.reset();

        // Define and Initialize Motors
        front_right    = hwMap.get(DcMotorEx.class,        "FR");
        front_left     = hwMap.get(DcMotorEx.class,        "FL");
        back_right     = hwMap.get(DcMotorEx.class,        "BR");
        back_left      = hwMap.get(DcMotorEx.class,        "BL");
        slide_1        = hwMap.get(DcMotorEx.class,        "S1");
        hook_1         = hwMap.get(DcMotorEx.class,        "H1");
        hook_2         = hwMap.get(DcMotorEx.class,        "H2");
        slide_2        = hwMap.get(DcMotorEx.class,        "S2");
        slide_3        = hwMap.get(Servo.class,            "S3");
        claw_2         = hwMap.get(Servo.class,            "C2");
        rotate_2       = hwMap.get(Servo.class,            "R2");
        rotate_1       = hwMap.get(Servo.class,            "R1");
        claw_1         = hwMap.get(Servo.class,            "C1");
        light_1        = hwMap.get(Servo.class,            "L1");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        //lifting_arm_2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_2.setDirection(DcMotorSimple.Direction.REVERSE);


        front_left .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setTargetPositionTolerance(20);
        front_right.setTargetPositionTolerance(20);
        back_left.setTargetPositionTolerance(20);
        back_right.setTargetPositionTolerance(20);

        slide_1  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hook_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lifting_arm_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //extend.setPosition(.2);

        //claw_left.setPosition(0.65);
    }

    // ====================
    // Section: Robot Modes
    // ====================

    public void teleOpMode() {
        front_left .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autonMode() {
        front_left .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ======================
    // Section: TeleOp Motion
    // ======================

    public void listen_for_drive_commands(double x) {
        double v = -GamePad1.left_stick_y;
        double h = GamePad1.left_stick_x;
        double p = GamePad1.right_stick_x / 2;

        double FR = (-p + (v - h) * x);
        double BR = (-p + (v + h) * x);
        double FL = ( p + (v + h) * x);
        double BL = ( p + (v - h) * x);

        FR = Range.clip(FR, -1, 1);
        FL = Range.clip(FL, -1, 1);
        BR = Range.clip(BR, -1, 1);
        BL = Range.clip(BL, -1, 1);

        front_left .setPower(FL);
        back_left  .setPower(BL);
        front_right.setPower(FR);
        back_right .setPower(BR);
    }

    public void moveslide(Integer value, double time_out) {
        runtime.reset();
        slide_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_1.setTargetPosition(value);
        slide_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_1.setPower(.3);
        while (motors_running(Collections.singletonList(slide_1)) && runtime.seconds() < time_out) {
            telemetry.addData("PixelArm", slide_1.getCurrentPosition());
            telemetry.update();
            listen_for_drive_commands(0.7);
        }
        slide_1.setPower(0);
    }

    public void moveLinerActuator(Integer value, double time_out) {
        runtime.reset();
        hook_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hook_2.setTargetPosition(value);
        hook_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hook_2.setPower(.3);
        while (motors_running(Collections.singletonList(hook_2)) && runtime.seconds() < time_out) {
            telemetry.addData("liner_actuator", hook_2.getCurrentPosition());
            telemetry.update();
            listen_for_drive_commands(0.7);
        }
        hook_2.setPower(0);
    }

    public void moveworm_gear(Integer value, double time_out) {
        runtime.reset();
        List<DcMotor> lift_arms = Arrays.asList(hook_1);
        for (int i = 0; i < lift_arms.size(); i++) {
            DcMotor motor = lift_arms.get(i);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(value);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        while (motors_running(lift_arms) && runtime.seconds() < time_out) {
            telemetry.addData("LiftingArm", hook_1.getCurrentPosition());
            //telemetry.addData("LiftingArm2", lifting_arm_2.getCurrentPosition());
            telemetry.update();
            listen_for_drive_commands(0.7);
        }
        for (DcMotor motor : lift_arms) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    // ==========================
    // Section: Autonomous Motion
    // ==========================

    public void rotate(double angle) {
        target = angle;
        if (angle > gyro.get_heading()){
            rotate_right(angle);
        }
        else if (angle < gyro.get_heading()) {
            rotate_left(angle);
        }
    }
    private void rotate_right(double angle) {
        teleOpMode();
        while (gyro.get_heading() <= angle) {
            double power = (Math.abs((gyro.get_heading() - angle) / 100) + 0.01);
            if (power > .5) {
                power = .5;
            }
            if (power < .15) {
                power = .15;
            }
            power += .1;
            front_right.setPower(power);
            front_left.setPower(-power);
            back_right.setPower(power);
            back_left.setPower(-power);
            telemetry.addData("heading", gyro.get_heading());
            telemetry.update();
        }
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
        autonMode();
    }
    private void rotate_left(double angle){
        teleOpMode();
        while (gyro.get_heading() >= angle) {
            double power = (Math.abs((gyro.get_heading() - angle) / 100) + 0.01);
            if (power > .5) {
                power = .5;
            }
            if (power < .15) {
                power = .15;
            }
            power += .1;
            front_right.setPower(-power);
            front_left.setPower(power);
            back_right.setPower(-power);
            back_left.setPower(power);
            telemetry.addData("heading", gyro.get_heading());
            telemetry.update();
        }
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
        autonMode();
    }

    public void drive(List<Integer> values, double original_power, boolean accelerating, double time_out) {
        runtime.reset();
        List<DcMotor> motors = Collections.unmodifiableList(
                Arrays.asList(front_right,
                        front_left,
                        back_right,
                        back_left));
        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(values.get(i));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (!accelerating) {
                motor.setPower(original_power);
            }
            else {
                motor.setPower(0.1);
            }
        }
        while (motors_running(motors) && runtime.seconds() < time_out){
            double absolute = gyro.get_heading();
            if (accelerating) {
                for (int i = 0; i < motors.size(); i++) {
                    DcMotor motor = motors.get(i);
                    telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
                    float value = values.get(i);
                    // float current = motor.getCurrentPosition();

                    double new_power;
                    if (value < 0) {
                        if (i % 2 == 0) {
                            new_power = original_power + (((absolute - target) / 100));
                        }
                        else {
                            new_power = original_power - (((absolute - target) / 100));
                        }

                    }
                    else {
                        if (i % 2 == 0) {
                            new_power = original_power - (((absolute - target) / 100));
                        }
                        else {
                            new_power = original_power + (((absolute - target) / 100));
                        }
                    }
                    motor.setPower(new_power);

                    //
                }
            }
            telemetry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public List<Integer> val(int v1, int v2, int v3, int v4) {
        return Collections.unmodifiableList(Arrays.asList(v1, v2, v3, v4));
    }

    public boolean motors_running(@NonNull List<DcMotor> motors) {
        boolean running = false;
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                running = true;
            }
        }
        return running;
    }
    public List<Double> power_val(double v1, double v2, double v3, double v4) {
        return Collections.unmodifiableList(Arrays.asList(v1, v2, v3, v4));
    }
}

