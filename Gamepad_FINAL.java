package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GamePad_FINAL", group = "")
public class Gamepad_FINAL extends LinearOpMode {

    DigitalChannel digitalTouch;  // Hardware Device Object;
    DigitalChannel DigitalTouch2;
    private DcMotor Right_Motor;
    private DcMotor Left_Motor;
    private DcMotor Latching_Motor_R;
    private DcMotor Latching_Motor_L;
    private CRServo Collector;
    private CRServo Arm0_Left;
    private CRServo Arm0_Right;
    private Servo Arm1;
    private long arm_run_duration = 40;
    private double arm_run_power = 1.0;
    private double arm_stop_power = 0.0;
    private boolean is_forward = true;
    private boolean is_power = false;

    private Servo S4;
    double s0_home = 0.7;
    double s1_home = 0.5;
    double s2_home = 0.6;
    double servo_increment = 0.002;
    double s0_min = 0.1;
    double s0_max = 1.0;
    double s1_min = 0.0;
    double s1_max = 0.8;
    boolean block_forward = false;
    boolean block_reverse = false;
    boolean just_reversed = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()  {
        Right_Motor = hardwareMap.dcMotor.get("Right_Motor");
        Left_Motor = hardwareMap.dcMotor.get("Left_Motor");
        Latching_Motor_L = hardwareMap.dcMotor.get("Latching_Motor_Right");
        Latching_Motor_R = hardwareMap.dcMotor.get("Latching_Motor_Left");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "Latch_Sensor");
        DigitalTouch2 = hardwareMap.get(DigitalChannel.class, "Latch_Sensor2");

        Arm0_Left = hardwareMap.crservo.get("S0");
        Arm0_Right = hardwareMap.crservo.get("S1");
        Arm1 = hardwareMap.servo.get("S2");
        Collector = hardwareMap.crservo.get("S3");

        // Put initialization blocks here.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        DigitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        Right_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Left_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Latching_Motor_L.setDirection(DcMotorSimple.Direction.FORWARD);
        Latching_Motor_R.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            Arm1.setPosition(s2_home);
            sleep(500);
            while (opModeIsActive()) {
                // Put loop blocks here.
                Left_Motor.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
                Right_Motor.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);

                // Landing
                if (gamepad1.left_bumper) {

                    if (!block_reverse) {
                        Latching_Motor_R.setDirection(DcMotorSimple.Direction.REVERSE);
                        Latching_Motor_L.setDirection(DcMotorSimple.Direction.REVERSE);
                        Latching_Motor_R.setPower(0.4);
                        Latching_Motor_L.setPower(0.4);
                        sleep(10);
                        Latching_Motor_L.setPower(0);
                        Latching_Motor_R.setPower(0);
                        sleep(20);
                        // check if latch reached bottom
                        if (digitalTouch.getState() == false && !just_reversed)
                        {

                            // block reverse direction to prevent latching from breaking
                            block_reverse = true;
                            block_forward = false;
                            just_reversed = true;
                        } else
                        {
                            just_reversed = false;
                        }
                    }
                }// We will not make our motors go reverse at this time because we are trying to latch on to it and not land
                if (gamepad1.right_bumper) {

                    if ( !block_forward) {
                        Latching_Motor_L.setDirection(DcMotorSimple.Direction.FORWARD);
                        Latching_Motor_R.setDirection(DcMotorSimple.Direction.FORWARD);
                        Latching_Motor_L.setPower(0.4);
                        Latching_Motor_R.setPower(0.4);
                        sleep(10);
                        Latching_Motor_L.setPower(0);
                        Latching_Motor_R.setPower(0);
                        sleep(20);
                        // check if latch reached bottom
                        if (DigitalTouch2.getState() == false && !just_reversed)
                        {
                            // block reverse direction to prevent latching from breaking
                            block_forward = true;
                            block_reverse = false;
                            just_reversed = true;
                        }else
                        {
                            just_reversed = false;
                        }
                    }
                }

                // Gamepad 2 Left Bumper moves S0 servo forward
                if (gamepad2.left_bumper) {
                    TurnArm(0, false);
                }
                // Gamepad 2 Right Bumper moves s0 servo reverse
                else if (gamepad2.right_bumper)
                {
                    TurnArm (0, true);
                }
                // Gampad 2 Left Trigger S1 forward
                else if (gamepad2.left_trigger > 0)
                {
                    telemetry.addLine();
                    telemetry.addData("Left Trigger ", " Pressed");
                    TurnArm (1, false);
                }
                // Gampad 2 Right Trigger S1 reverse
                else if (gamepad2.right_trigger > 0)
                {
                    TurnArm (1, true);
                }
                else if (gamepad2.a)
                {
                    if (is_power) {
                        is_power = false;
                        Collector.setPower(0);
                    }else{
                        if (is_forward) {
                            Collector.setDirection(DcMotorSimple.Direction.FORWARD);
                        }else {
                            Collector.setDirection(DcMotorSimple.Direction.REVERSE);
                        }
                        Collector.setPower(0.7);
                    }
                }
                else if (gamepad2.b)
                {
                    if (is_forward)
                    {
                        is_forward = false;

                    } else
                    {
                        is_forward = true;
                    }
                }
                telemetry.addLine();
                telemetry.addData("Updated controls ", "v1");
                telemetry.addLine();
                telemetry.addData("S2 Position ", Arm1.getPosition());
                telemetry.addLine();
                if (digitalTouch.getState() == true) {
                    telemetry.addData("Latch Sensor", "Is Not Pressed");
                } else
                {
                    telemetry.addData("Latch Sensor", "Pressed");
                }
                telemetry.update();

            }
        }
    }


    public void TurnArm(int s_index, boolean forward)
    {
        switch (s_index) {
            case 0: {
                if (forward) {
                    Arm0_Left.setPower(arm_run_power);
                    Arm0_Right.setPower(-1*arm_run_power);
                } else
                {
                    Arm0_Left.setPower(-1*arm_run_power);
                    Arm0_Right.setPower(arm_run_power);

                }
                sleep(arm_run_duration);
                Arm0_Left.setPower(arm_stop_power);
                Arm0_Right.setPower(arm_stop_power);

                break;
            }
            case 1: {
                double pos = Arm1.getPosition();
                if (forward) {
                    pos = pos + servo_increment;
                    if (pos >= s1_max)
                    {
                        pos = s1_max;
                    }
                }
                else {
                    pos = pos - servo_increment;
                    if (pos <= s1_min)
                    {
                        pos = s1_min;
                    }

                }
                Arm1.setPosition(pos);
                break;
            }
        }
    }
}



