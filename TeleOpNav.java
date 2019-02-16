package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * This is the base class from teleOpMoongoose
 * This class contains the calling of motors, servos, sensors, variables for TeleOp and the automation methods for the hopper lift and the lift
 */
public class TeleOpNav {
    //Drivetrain motors//
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //Motors for mechanisms//
    public DcMotor extendy;            // lead screw horizontal extension
    public DcMotor lifty;              // lift for hanging
    public DcMotor liftyJr;            // lift for hopper and deposit into
    public DcMotor collecty;           // collection intake

    //Servos//
    public Servo teamMarker;           // team Marker servo (only used in teleOp in case of emergency
    public Servo droppy;               // intake position servo (right)
    public Servo droppyJr;             // intake position servo (left)
    public TouchSensor limitSwitch;    // touch sensor for liftyJr -- hopper lift, placed on bottom of lift
 //   public TouchSensor extendySwitch;  // touch sensor for extendy -- horizontal extension, placed on back of robot

    //Variables//
    public double calibToggle, driveToggle, liftToggle;
    public double liftySpeed, liftyJrSpeed;
    public double normalSpeed, slowSpeed;

    public boolean canMoveLiftyJr, liftylike;

    public int driveSpeed, driveMode, liftMode;
    public int manualMode;

    //Objects//
    public ElapsedTime period = new ElapsedTime();
//
    public void init(HardwareMap hwMap) {
        //Drivetrain Motors//
        backLeft = hwMap.dcMotor.get("backLeft");
        backRight = hwMap.dcMotor.get("backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Motors For Mechanisms//
        lifty = hwMap.dcMotor.get("lifty");
        liftyJr = hwMap.dcMotor.get("liftyJr");

        lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftyJr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Servos//
        collecty = hwMap.dcMotor.get("collecty");
        droppy = hwMap.servo.get("droppy");
        droppyJr = hwMap.servo.get("droppyJr");
        teamMarker = hwMap.servo.get("teamMarker");


        droppyJr.setDirection(Servo.Direction.REVERSE);
        extendy.setDirection(DcMotor.Direction.REVERSE);
        teamMarker.setPosition(0.2);

        //Sensors//
        limitSwitch = hwMap.touchSensor.get("limitSwitch");
   //     extendySwitch = hwMap.touchSensor.get("extendySwitch");

        //Setting Variables//
        calibToggle = 0;
        canMoveLiftyJr = true;
        driveToggle = 0;
        driveSpeed = 0;
        driveMode = 0;

        //Speed Offsets//
        normalSpeed = .80;
        liftySpeed = 1;
        liftyJrSpeed = 1;
        slowSpeed = normalSpeed / 2;

    }

    public void setCanMove(Boolean canMove) {
        canMoveLiftyJr = canMove;
    }

    public void resetEncoders() {
        liftyJr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

/////// AUTOMATION FOR TELEOP //////////

//HOPPER LIFT//

    // This method brings the hopper lift back to the lowest position possible (usually on the limit switch)
    public void goDown(){
        if (!limitSwitch.isPressed()){
            liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftyJr.setTargetPosition(-2);
            liftyJr.setPower(1); } }
    // This method goes up using an encoder to where the hopper lift is under the bar only slighty in preparation for dropping
    public void goUpBit() {
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setPower(1);
        liftyJr.setTargetPosition(-1500); }
    // This method goes up using an encoder for the hopper lift to fully dump into the lander
    public void goUpAll() {
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setTargetPosition(-2065);
        liftyJr.setPower(1); }
    public void goupBalance(){
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setTargetPosition(-630);
        liftyJr.setPower(1);
    }

//HANGING LIFT//
    // This method raises the hanging lift to in between the handle
    public final void ITS_ENDGAME_NOW() {
        lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty.setPower(1);
        lifty.setTargetPosition(10464); }

//HORIZONTAL EXTENSION//
    public void retract(){
        extendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendy.setPower(1);
        extendy.setTargetPosition(15); }


    public double round(double value) { //Allows telemetry to display nicely
        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(3, RoundingMode.HALF_UP);
        return bd.doubleValue(); }

}
