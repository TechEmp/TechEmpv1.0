package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A class to run Autonomous given a strategy with only encoders. This is our "first" iteration of autonomous and has been further developed
 * into AutoGeneric. This class is not longer used during competition, by hypothically it could if the imu system stopped working on both of the
 * Rev Expansion Hubs on the robot.
 */
public class AutoBackupFile {

    public enum StartPos {DEPOTAUTO, CRATERAUTO, DOUBLESAMPLINGAUTO}

    private StartPos startZone;
    private OpMode opMode;
    private Telemetry telemetry;
    private Navigation nav;

    /**
     * The constructor method that contains everything to run in initialization.
     *
     * @param startZone - StartPos enumerator. Tells which strategy to run. Options are DEPOT, CRATER, or DOUBLESAMPLING.
     * @param opMode    - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry - Telemetry of the current OpMode, used to output data to the screen.
     */
    public AutoBackupFile(AutoBackupFile.StartPos startZone, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.startZone = startZone;
        this.opMode = opMode;
        this.telemetry = telemetry;
        nav = new Navigation(opMode, telemetry,true);
        nav.hold(0.1f);
    }

    // Run this to run Autonomous. //
    public void runOpMode() {
        nav.updateCubePos();
        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
        nav.holdForLift();
        nav.goDistanceHold(3f);
        nav.pointTurnRelative(-90f);
        nav.holdForDrive();
        nav.setLiftHeight(Navigation.LiftHeight.LOWER); //Lowering lift back to starting position
        nav.goDistanceHold(13f);
        switch (nav.getCubePos()) { //all of them for sampling
            case MIDDLE:
                nav.goDistanceHold(12f); //less forward (same total distance as before)
                nav.goDistanceHold(-12f);  //same distance back
                nav.pointTurnRelative(80f); //90 degrees to left
                break;
            case RIGHT:
                nav.pointTurnRelative(-60f); //turning 5 degrees more
                nav.holdForDrive();
                nav.goDistanceHold(20f); //going same distance forward and back
                nav.goDistanceHold(-20f);
                nav.pointTurnRelative(140f); //turning total 90
                break;
            default:
                nav.pointTurnRelative(55f);
                nav.holdForDrive();
                nav.goDistanceHold(15f);
                nav.goDistanceHold(-15f);
                nav.pointTurnRelative(25f); //turning total of 90
                break; }
        nav.holdForDrive();
        //-----crater depot run-----//
        if (startZone == StartPos.CRATERAUTO) {
            nav.goDistanceHold(44f);
            nav.pointTurnRelative(-125f);
            nav.holdForDrive();
}
        // depot side //
        else if (startZone == StartPos.DEPOTAUTO) {
            nav.goDistanceHold(45f);
            nav.pointTurnRelative(43f); //want a little bit more for gliding on the wall
            nav.holdForDrive();
            nav.goDistanceHold(-35f); }

        //-----crater doublesampling------//
        else if (startZone == StartPos.DOUBLESAMPLINGAUTO) {
            nav.goDistanceHold(44f);
            nav.pointTurnRelative(-128f);
            nav.holdForDrive();
            nav.curveTurn(-40f,10f,0f,15f);
            switch (nav.getCubePos()) {
                case MIDDLE:
                    nav.pointTurnRelative(-90f);
                    nav.holdForDrive();
                    nav.goDistanceHold(30f);
                    nav.goDistanceHold(-30f);
                    nav.pointTurnRelative(90f);
                    break;
                case RIGHT:
                    nav.pointTurnRelative(-45f);
                    nav.holdForDrive();
                    nav.goDistanceHold(30f);
                    nav.goDistanceHold(-30f);
                    nav.pointTurnRelative(45f);
                    break;
                default: //Left
                    nav.pointTurnRelative(-135f);
                    nav.holdForDrive();
                    nav.goDistanceHold(30f);
                    nav.goDistanceHold(-30f);
                    nav.pointTurnRelative(135f);
                    nav.holdForDrive(); }

                nav.curveTurn(10f,-11f,0f,0f); }

        //-----marker deploy and driving to crater-----//
        nav.setTeamMarker(0.8f);
        nav.hold(1);
        nav.goDistanceHold(60f);
        nav.hold(2);
    }
}