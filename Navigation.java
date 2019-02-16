package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.io.File;
import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * A class for all movement methods (using PID and IMU) for Rover Ruckus for autonomous as well as mechanisms methods for autonomous as well
 * (Basically an autonomous base)
 */
public class Navigation {

    //-----tweak values-----//
    //private float maximumMotorPower = 0.5f;             //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float encoderCountsPerRev = 537.6f;         //encoder ticks per one revolution
    private boolean useTelemetry;                       //whether to execute the telemetry method while holding
    private float minVelocityCutoff = 0.06f;            //velocity with which to continue program execution during a hold (encoder ticks per millisecond)

    //-----enums-----//
    public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
    private CubePosition cubePos = CubePosition.UNKNOWN;
    public enum CollectorHeight {COLLECT, HOLD, LAND, DUMP}
    public enum LiftHeight {LOWER, HOOK}
    public enum LiftyJrHeight {LOWER, DROP, WAIT, BALANCE}
    public enum CollectorExtension {PARK, DUMP, OUT}
    public enum CollectorSweeper {INTAKE,OUTTAKE, OFF}

    //-----misc internal values-----//
    private com.qualcomm.robotcore.eventloop.opmode.LinearOpMode hardwareGetter;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private Dogeforia vuforia;
    private GoldAlignDetector detector;
    private WebcamName webcamName;
    private VuforiaTrackables vumarks;
    private DcMotor velocityMotor;
    private long prevTime;
    private int prevEncoder;
    private float velocity = 0f;
    private Location[] vumarkLocations = new Location[4];
    private Location camLocation = new Location(0f, 6f, 6f, 0f);
    private float wheelDistance = 6.66f;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)
    private Location pos = new Location();
    BNO055IMU imu;
    public Orientation angles;
    public int gameState = 0;

    //location of robot as [x,y,z,rot] (inches / degrees)

    //-----motors-----//

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lifty;  //lift motor a
    private DcMotor liftyJr; //lift motor b

    //-----servos-----//

    private Servo teamMarker;



    /**
     * The constructor class for Navigation
     * @param hardwareGetter - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry      - Telemetry of the current OpMode, used to output data to the screen.
     * @param useTelemetry   - Whether or not to output information about stored variables and motors during hold periods.
     */
    public Navigation(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.useTelemetry = useTelemetry;

        //-----motors-----//
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("MPL");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("MPR");
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /**        extendy.setPower(1f);
         extendy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         extendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */

        lifty = hardwareGetter.hardwareMap.dcMotor.get("MLL");
        lifty.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty.setDirection(DcMotor.Direction.FORWARD);
        lifty.setPower(1);

        liftyJr = hardwareGetter.hardwareMap.dcMotor.get("MLR");
        liftyJr.setDirection(DcMotor.Direction.FORWARD);
        liftyJr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setPower(1);
        setLiftHeight(0);


        //-----servos-----//
        teamMarker = hardwareGetter.hardwareMap.servo.get("SD");




        //----Vuforia Params---///
        webcamName   = hardwareGetter.hardwareMap.get(WebcamName.class, "MyWebcam");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.vuforiaLicenseKey = "ATa8KrP/////AAABmVURXyq7wkdruDVzDWMKmjsuo6eVXMayj0l5ym4aLLMOM+Gb/NUAKqV2ke5Pvfh16adRgriNdWCQdY+RDLK7BZ1XTu0A5yORPb0BNjSbIGQLgA7/gzwAS3EX/1tMKI5y64DMZ7lu2oIEyCxzsGGLXH0A/H3nnSw2zyCIextTuS5OzF1I9ILCZS3pXnE/OTPC1N3qfMozeHN3sGNBsTSe/1sDg28FlpqluCJVbWONbJkPOoH+WY3cZdQoYxWC5fXdVgmIaUQq1EGvOwwFvLdV5ceNMHCbbM+T++xQ0F4AsHiScCyjh62Kj5V8Cb41YclIR1xol2HNzd1+Hb6HzVWxDcnNjKFGBawlSLg6kEO42SmQ\n";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();


        //Setup trackables
        vumarks = vuforia.loadTrackablesFromAsset("18-19_rover_ruckus");
        vumarkLocations[0] = new Location(0f, 5.75f, 71.5f, 180f); //east
        vumarkLocations[1] = new Location(-71.5f, 5.75f, 0f, 270f); //north
        vumarkLocations[2] = new Location(0f, 5.75f, -71.5f, 0f); //west
        vumarkLocations[3] = new Location(71.5f, 5.75f, 0f, 90f); //south
        vumarks.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareGetter.hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        //-----velocity control-----//
        prevTime = System.currentTimeMillis();
        prevEncoder = velocityMotor.getCurrentPosition();

        //---imu initialization-----//
        BNO055IMU.Parameters noots = new BNO055IMU.Parameters();
        noots.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        noots.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        noots.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        noots.loggingEnabled = true;
        noots.loggingTag = "IMU";
        noots.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareGetter.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(noots);

        calibrateHeading();
    }

    /**
     * Updates the Robot's position using Vuforia. Value is in inches from center of map (see ccoordinate_diagram.png). Access using [nav].pos.
     * @return boolean, true if updated, false otherwise
     */
    public boolean updatePos() {

        for (int i = 0; i < vumarks.size(); i++) {
            OpenGLMatrix testLocation = ((VuforiaTrackableDefaultListener) vumarks.get(i).getListener()).getPose();
            if (testLocation != null) {
                Location markLocation = new Location(vumarkLocations[i].getLocation(0), vumarkLocations[i].getLocation(1), vumarkLocations[i].getLocation(2), vumarkLocations[i].getLocation(3) - (float) Math.toDegrees(testLocation.get(1, 2)));
                markLocation.translateLocal(testLocation.getTranslation().get(1), -testLocation.getTranslation().get(0), testLocation.getTranslation().get(2));
                markLocation.translateLocal(camLocation.getLocation(0), camLocation.getLocation(1), camLocation.getLocation(2));
                markLocation.setRotation(markLocation.getLocation(3) + 180f);
                pos = markLocation;
                return true;
            }
        }
        return false;
    }

    /**
     * @return Location of the robot as Location object.
     */
    public Location getPos(){return pos;}

    /**
     * Updates the cube location enumerator using OpenCV. Access using [nav].cubePos.
     * @return boolean, true if updated, false if not updated.
     */
    public boolean updateCubePos() {

        double cubeX = detector.getXPosition();
        //unknown
        if (cubeX == 0.0) {
            return false;
        }
        //left
        else if (cubeX < 170) {
            cubePos = CubePosition.LEFT;
        }
        //middle
        else if (cubeX < 400) {
            cubePos = CubePosition.MIDDLE;
        }
        //right
        else {
            cubePos = CubePosition.RIGHT;
        }


        return true;
    }

    /**
     * Returns the current stored cube position.
     * @return CubePosition enumerator. Locations self-explanatory.
     */
    public CubePosition getCubePos() {
        return cubePos;
    }


    /**
     * Sets drive motor powers.
     *
     * @param left  power of left two motors as percentage (0-1).
     * @param right power of right two motors as percentage (0-1).
     */
    public void drivePower(float left, float right) {
        backRight.setPower(right);
        backLeft.setPower(left);
    }

    /**
     * Sets drive motor target encoder to given values.
     * @param left  encoder set for left motors.
     * @param right encoder set for right motors.
     */
    public void drivePosition(int left, int right) {
        backRight.setTargetPosition(right);
        backLeft.setTargetPosition(left);
    }

    /**
     * Sets all drive motor run modes to given mode.
     * @param mode name DcMotor mode to given value.
     */
    public void driveMode(DcMotor.RunMode mode) {
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }

    /**
     * Stops all drive motors and resets encoders.
     */
    public void stopAllMotors() {
        drivePower(0f, 0f);
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Pseudo PID to drive the given distance.
     * @param distance Distance to drive forward in inches.
     */
    public void goDistance(float distance, float maximumMotorPower, float maxiumMotorPower) {
        driveMethodSimple(-distance, distance, maximumMotorPower, maximumMotorPower);
        pos.translateLocal(distance);
    }

    /**
     * Same as goDistance() e.g. PID drive in a straight line, but with a holdForDrive() at the end. I'm not sure. Just roll with it.
     * @param distance Distance to drive forward in inches.
     */

    public void goDistanceHold(float distance){
        goDistance(distance, 0.55f, 0.55f);
        holdForDrive();
    }

    public void setLiftJrHeight(int position) {
        liftyJr.setTargetPosition(position);
    }

    public void setLiftyJrHeight(LiftyJrHeight position) {
        switch (position) {
            case LOWER:
                setLiftJrHeight(0);
                break;
            case DROP:
                setLiftJrHeight(2050);
                break;
            case WAIT:
                setLiftJrHeight(1500);
            case BALANCE:
                setLiftJrHeight(630);
                break;

        }
    }

    /**
     * Executes a point turn to face the given world rotation.
     * @param rot Target azimuth in degrees
     */
    public void pointTurn(float rot) {
        float rota = (rot - pos.getLocation(3)) % 360f;
        float rotb = -(360f - rota);
        float optimalRotation = (Math.abs(rota) < Math.abs(rotb) ? rota : rotb); //selects shorter rotation
        float distance = (float) (Math.toRadians(optimalRotation) * wheelDistance); //arc length of turn (radians * radius)
        driveMethodSimple(distance, distance, 0.3f, 0.3f);

        pos.setRotation(rot);
    }

    /**
     * Executes a point turn to face the given Location.
     * @param loc Target Location object
     */
    public void pointTurn(Location loc) {
        pointTurn((float) Math.toDegrees(Math.atan2(loc.getLocation(2) - pos.getLocation(2), loc.getLocation(0) - pos.getLocation(0))));
    }

    /**
     * Executes a point turn relative to the current location. Positive is counterclockwise.
     * @param rot the amount to rotate the robot in degrees. Positive is counterclockwise.
     */
    public void pointTurnRelative(float rot) {
        pointTurn(pos.getLocation(3) + rot);
    }

    /**
     * Executes a point turn to the given heading, first updating the position with the internal IMU value. Will holdForDrive() automatically.
     * @param heading Target rotation in degrees.
     */
    public void pointTurnIMU(float heading) {
        pos.setRotation((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle);
        pointTurn(heading);
        holdForDrive();
    }

    /**
     * Rotate the robot smothly around a curve, with differing wheel rates for each side.
     * @param distance1 Distance to travel straight before curve (inches)
     * @param distanceL Distance for left side of robot to travel (inches)
     * @param distanceR Distance for right side of robot to travel (inches)
     * @param distance2 Distance to travel straight after the curve (inches)
     */
    public void curveTurn(float distance1,float distanceL, float distanceR, float distance2){
        goDistance(distance1, 0.55f, 0.55f);
        holdForDrive();
        driveMethodSimple(distanceL, distanceR, 0.3f, 0.3f);
        holdForDrive();
        goDistance(distance2, 0.55f, 0.55f);
        holdForDrive();

        pos.setRotation((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle);
    }

    /**
     * Sets lift motor to given encoder position
     * @param position Encoder ticks for lift motor. ~0(bottom) to ~8200(top)
     */
    public void setLiftHeight(int position) {
        lifty.setTargetPosition(position);
    }

    /**
     * Sets the lift height to a pre-programmed position.
     * @param position LiftHeight enumerator. Options are LOWER, HOOK, or SCORE.
     */
    public void setLiftHeight(LiftHeight position) {
        switch (position) {
            case HOOK:
                setLiftHeight(-10464);
                break;
            case LOWER:
                setLiftHeight(0);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
                break;
        }
    }

    /**
     * Sets the collection sweeper to a given power value.
     * @param power float. Percentage power at which to run collector. 1.0f (intake) - -1.0f(outtake) inclusive.
     */


    /**
     * Sets the collection sweeper power using pre-programmed values.
     * @param power CollectorSweeper emumerator. Options are INTAKE, OUTTAKE, or OFF.
     */




    /**
     * Set the height of the collector arm.
     * @param position float. ~0.8f (bottom) to ~0.18f (top).
     */

    /**
     * Sets the height of the collector arm.
     * @param position CollectorHeight enumerator. Options are COLLECT, HOLD, or DUMP.
     */

    /**
     * Sets the extension of the collector arm.
     * @param position int. 0(in) to 1600(out).

    public void setCollectorExtension(int position) {
    extendy.setTargetPosition(position);
    }
     */
    /**
     * Sets the extension of the collectior arm.
     * @param position CollectorExtension enumerator. Options are PARK, DUMP, or OUT.
     */
    /**  public void setCollectorExtension(CollectorExtension position) {
     switch (position) {
     case PARK:
     setCollectorExtension(0);
     break;
     case DUMP:
     setCollectorExtension(500);
     break;
     case OUT:
     setCollectorExtension(1600);
     break;
     }
     }
     */

    /**
     * Sets the position of the teamMarker servo.
     * @param position float. 0.0f(locked) to 0.8f(dropping).
     */
    public void setTeamMarker(float position) {
        teamMarker.setPosition(position);
    }

    /**
     * Drive method that independently controls the position and power of the left and right drive motors.
     * @param distanceL float. Distance in inches for left motors to traverse.
     * @param distanceR float. Distance in inches for right motors to traverse.
     * @param LPower float. Power percentage for left motors (0.0-1.0).
     * @param RPower float. Power percentage for right motors (0.0-1.0).
     */
    private void driveMethodSimple(float distanceL, float distanceR, float LPower, float RPower) {
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int l = (int)(distanceL / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int r = (int)(distanceR / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        drivePosition(-l,-r);
        drivePower(LPower,RPower);
        driveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Holds program execution until drive motor velocities are below the minimum cutoff.
     * Will output telemetry if class initialized with useTelemetry true.
     */
    public void holdForDrive() {
        hold(0.2f);
        gameState++;
        while (updateVelocity() > minVelocityCutoff && hardwareGetter.opModeIsActive()) {
            if (useTelemetry) telemetryMethod();
        }
    }

    /**
     * Holds program execution until lift motor is done moving.
     * Will output telemetry if class initialized with useTelemetry true.
     */
    public void holdForLift() {
        hold(0.1f);
        gameState++;
        while (lifty.isBusy() && hardwareGetter.opModeIsActive()) {
            if (useTelemetry) telemetryMethod();
        }
    }

    public void holdForLiftJr() {
        hold(0.1f);
        gameState++;
        while (liftyJr.isBusy() && hardwareGetter.opModeIsActive()) {
            if (useTelemetry) telemetryMethod();
        }
    }

    /**
     * Hold program for given number of seconds.
     * @param seconds float. Number of seconds to wait.
     */
    public void hold(float seconds) {
        long stopTime = System.currentTimeMillis() + (long) (seconds * 1000);
        gameState++;
        while (System.currentTimeMillis() < stopTime && hardwareGetter.opModeIsActive()) {
            if (useTelemetry) telemetryMethod();
        }
    }

    /**
     * Updates the stored velocity of the robot to reflect reality.
     * @return float. New velocity in encoder ticks per millisecond.
     */
    private float updateVelocity() {
        velocity = Math.abs((float) (velocityMotor.getCurrentPosition() - prevEncoder) / (System.currentTimeMillis() - prevTime));
        prevEncoder = velocityMotor.getCurrentPosition();
        prevTime = System.currentTimeMillis();
        return velocity;
    }

    /**
     * A simple method to output the status of all motors and other variables to telemetry.
     */
    public void telemetryMethod () {
        updateVelocity();
        telemetry.addData("Game State = ", gameState);
        String motorString = "FL = " + " BL = " + backLeft.getCurrentPosition() + " FR = " +  " BR = " + backRight.getCurrentPosition();
        telemetry.addData("Drive = ", motorString);
        telemetry.addData("Lift = ", lifty.getCurrentPosition());
        telemetry.addData("Collector L/E/C = ", lifty.getCurrentPosition() + " " + " " );
        telemetry.addData("Pos = ", pos);
        telemetry.addData("CubePos = ", cubePos);
        telemetry.addData("Velocity = ", velocity);
        //   telemetry.addData("CubeXPosition",detector.getXPosition());
        telemetry.update();
    }

    /**
     * Calibrates the imu, probably best to do in init
     * May take a hot second.
     */
    public void calibrateHeading() {
        BNO055IMU.Parameters noots = new BNO055IMU.Parameters();
        noots.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        noots.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        noots.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        noots.loggingEnabled = true;
        noots.loggingTag = "IMU";
        noots.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(noots);

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "BNO055IMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());

        pos.setRotation((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle);

        telemetry.update();
        telemetry.log().add("IMU: CALIBRATED", filename);
        telemetry.update();
    }

    /**
     * Compares given heading value with IMU heading value. If less than error, returns true.
     * @param heading the heading to check for, heading in is degrees
     * @param err the amount of error (in degrees) allowed to return true
     * @return boolean.
     */
    public boolean checkHeading(float heading, float err) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Math.abs(heading - angles.firstAngle) < err;
    }

    private static double round(double value) { //Allows telemetry to display nicely
        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(3, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
