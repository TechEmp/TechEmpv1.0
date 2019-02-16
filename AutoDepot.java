package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A class made to run the depot code pulled from AutoGeneric
 */
@Autonomous(name="Auto Depot", group="Auto")
public class AutoDepot extends LinearOpMode {
    public void runOpMode() {
        AutoGeneric autoGeneric = new AutoGeneric(AutoGeneric.StartPos.DEPOT, this, telemetry);
        while(!isStarted()){
            telemetry.addData("cool","waiting to start");
            telemetry.update();
        }
       // waitForStart();

        opModeIsActive();

        autoGeneric.runAutonomous();
    }

    public boolean isStopping() {
        return opModeIsActive();
    }
}
