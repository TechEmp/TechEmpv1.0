package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * This is a testing class (specifically with drive methods from autonomous)
 */
@Autonomous(name="Test Nav Methods")
public class TestNavMethods extends LinearOpMode {
    private Navigation nav;
    @Override

    public void runOpMode(){

        nav = new Navigation(this, telemetry, true);
        waitForStart();
        while (opModeIsActive()) {
        nav.updatePos();
        telemetry.addData("Position", nav.getPos());
        nav.pointTurnIMU(15f);
        nav.pointTurnIMU(-15f);
        nav.pointTurnIMU(25f);
        nav.pointTurnIMU(-25f);
        nav.pointTurnIMU(45f);
        nav.pointTurnIMU(-45f);
        }

    }
//

//      nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
//        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
//        nav.setCollectorExtension(Navigation.CollectorExtension.PARK);
//
//        nav.setLiftHeight(Navigation.LiftHeight.SCORE);
//        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
//        nav.setLiftHeight(Navigation.LiftHeight.LOWER);

//        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);
//        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT);
//        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
//        nav.hold(3f);
//        nav.setCollectorHeight(Navigation.CollectorHeight.HOLD);
//        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
//        nav.hold(3f);
//        nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
//        nav.hold(3f);
//        nav.setCollectionSweeper(Navigation.CollectorSweeper.OFF);

//        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);
//        nav.setCollectionSweeper(Navigation.CollectorSweeper.OUTTAKE);


    }


