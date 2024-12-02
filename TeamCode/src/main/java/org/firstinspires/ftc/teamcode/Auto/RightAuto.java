package org.firstinspires.ftc.teamcode.Auto;//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Globals.robotMap;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
//import org.firstinspires.ftc.teamcode.system_controllers.clawController;
//import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
//import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
//import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
//import org.firstinspires.ftc.teamcode.system_controllers.liftController;
//import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
//import org.firstinspires.ftc.teamcode.system_controllers.transferController;
//
//@Config
//@Autonomous
//public class RightAuto extends LinearOpMode {
//
//    enum STROBOT {
//        START,
//        PRELOAD,
//        PRELOAD_SCORE,
//        PRELOAD_CLAW,
//        GO_TO_COLLECT,
//        GO_TO_OUTTAKE_DOWN,
//        COLLECT_SAMPLE,
//        COLLECT_EXTEND,
//        COLLECT_DROP,
//        COLLECT_VERIF,
//        SAMPLE_DROP,
//        FOURBAR,
//        SPIT,
//        SAMPLE_VERIF,
//        GO_TO_POSE,
//        CLAW_DROP,
//        OUTTAKE_DOWN,
//        GO_TO_COLLECT_SPECIMEN,
//        PREPARE_COLLECT,
//        VERIF_CLAW,
//        FOURBAR_UP,
//        SCORE_SPECIMEN,
//        SCORE,
//        OUTTAKE_SCORE,
//        SPECIMEN_CLAW,
//        GO_TO_SPECIMEN_COLLECT,
//        GET_OUTTAKE_DOWN,
//        GET_OUTTAKE_DOWN2,
//        PARK,
//        NOTHING
//    }
//
//    public Telemetry telemetryA;
//
//    public static double sample1Angle = Math.toRadians(120);
//    public static double sample2Angle = Math.toRadians(0);
//    public static double sample3Angle = Math.toRadians(20);
//
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Follower drive = new Follower(hardwareMap);
//        robotMap r = new robotMap(hardwareMap);
//
//        /**
//         * POSES
//         */
//
//        Pose startPose = new Pose(8.5,66,Math.toRadians(0));
//        Pose preloadPose = new Pose(40,70,Math.toRadians(0));
//        Pose collectSamplePose = new Pose(36,42,Math.toRadians(0));
//        Pose sample1Pose = new Pose(36,42,sample1Angle);
//        Pose sample2Pose = new Pose(36,42,sample2Angle);
//        Pose sample3Pose = new Pose(36,42,sample3Angle);
//        Pose collectSpecimenPose = new Pose(36,42,Math.toRadians(0));
//        Pose scoreSpecimen1Pose = new Pose(40,70,Math.toRadians(0));
//        Pose scoreSpecimen2Pose = new Pose(40,69,Math.toRadians(0));
//        Pose scoreSpecimen3Pose = new Pose(40,68,Math.toRadians(0));
//        Pose scoreSpecimen4Pose = new Pose(40,67,Math.toRadians(0));
//        Pose scoreSpecimen5Pose = new Pose(40,66,Math.toRadians(0));
//
//        /**
//         * PATHS
//         */
//
//        Path preload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
//        preload.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectSample = new Path(new BezierLine(new Point(preloadPose), new Point(sample1Pose)));
//        collectSample.setLinearHeadingInterpolation(Math.toRadians(0),sample1Angle);
//
//        Path sample1 = new Path(new BezierLine(new Point(collectSamplePose), new Point(sample1Pose)));
//        sample1.setLinearHeadingInterpolation(Math.toRadians(0),sample1Angle);
//
//        Path sample2 = new Path(new BezierLine(new Point(sample1Pose), new Point(sample2Pose)));
//        sample2.setLinearHeadingInterpolation(-sample1Angle,sample2Angle);
//
//        Path sample3 = new Path(new BezierLine(new Point(sample2Pose), new Point(sample3Pose)));
//        sample3.setLinearHeadingInterpolation(-sample2Angle,sample3Angle);
//
//        Path dropSample1 = new Path(new BezierLine(new Point(sample1Pose), new Point(collectSamplePose)));
//        dropSample1.setLinearHeadingInterpolation(sample1Angle,-sample1Angle);
//
//        Path dropSample2 = new Path(new BezierLine(new Point(sample2Pose), new Point(collectSamplePose)));
//        dropSample2.setLinearHeadingInterpolation(sample2Angle,-sample2Angle);
//
//        Path dropSample3 = new Path(new BezierLine(new Point(sample3Pose), new Point(collectSamplePose)));
//        dropSample3.setLinearHeadingInterpolation(sample3Angle,-sample3Angle);
//
//        Path scoreSpecimen1 = new Path(new BezierLine(new Point(collectSpecimenPose), new Point(scoreSpecimen1Pose)));
//        scoreSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path scoreSpecimen2 = new Path(new BezierLine(new Point(collectSpecimenPose), new Point(scoreSpecimen2Pose)));
//        scoreSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path scoreSpecimen3 = new Path(new BezierLine(new Point(collectSpecimenPose), new Point(scoreSpecimen3Pose)));
//        scoreSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path scoreSpecimen4 = new Path(new BezierLine(new Point(collectSpecimenPose), new Point(scoreSpecimen4Pose)));
//        scoreSpecimen4.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path scoreSpecimen5 = new Path(new BezierLine(new Point(collectSpecimenPose), new Point(scoreSpecimen5Pose)));
//        scoreSpecimen5.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectSpecimen1 = new Path(new BezierLine(new Point(scoreSpecimen1Pose), new Point(collectSpecimenPose)));
//        collectSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectSpecimen2 = new Path(new BezierLine(new Point(scoreSpecimen2Pose), new Point(collectSpecimenPose)));
//        collectSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectSpecimen3 = new Path(new BezierLine(new Point(scoreSpecimen3Pose), new Point(collectSpecimenPose)));
//        collectSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectSpecimen4 = new Path(new BezierLine(new Point(scoreSpecimen4Pose), new Point(collectSpecimenPose)));
//        collectSpecimen4.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectSpecimen5 = new Path(new BezierLine(new Point(scoreSpecimen5Pose), new Point(collectSpecimenPose)));
//        collectSpecimen5.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        Path collectFirstSpecimen = new Path(new BezierLine(new Point(collectSamplePose), new Point(collectSpecimenPose)));
//        collectFirstSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        /**
//         * INITS
//         */
//
//        clawController claw = new clawController();
//        clawAngleController clawAngle = new clawAngleController();
//        collectAngleController collectAngle = new collectAngleController();
//        extendoController extendo = new extendoController();
//        fourbarController fourbar = new fourbarController();
//        liftController lift = new liftController();
//        outtakeController outtake = new outtakeController();
//        transferController transfer = new transferController();
//
//        double voltage;
//        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//        voltage = batteryVoltageSensor.getVoltage();
//
//        claw.CS = clawController.clawStatus.CLOSED;
//        clawAngle.CS = clawAngleController.clawAngleStatus.DRIVE;
//        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//        extendo.CS = extendoController.extendoStatus.RETRACTED;
//        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
//        lift.CS = liftController.liftStatus.DOWN;
//        outtake.CS = outtakeController.outtakeStatus.DRIVE;
//
//        /**
//         * UPDATES
//         */
//
//        claw.update(r);
//        clawAngle.update(r);
//        collectAngle.update(r);
//        extendo.update(r,0,0.5,voltage);
//        fourbar.update(r);
//        lift.update(r,0,voltage);
//        outtake.update(r,claw,lift,fourbar,clawAngle,transfer);
//
//
//
//        drive.setPose(startPose);
//        STROBOT status = STROBOT.START;
//
//        ElapsedTime timer = new ElapsedTime();
//        ElapsedTime timer_preload = new ElapsedTime();
//        ElapsedTime timer_wait = new ElapsedTime();
//        ElapsedTime timer_pleaca = new ElapsedTime();
//        ElapsedTime timer_down = new ElapsedTime();
//        ElapsedTime timer_fourbar = new ElapsedTime();
//        ElapsedTime timer_claw = new ElapsedTime();
//        ElapsedTime timer_specimen = new ElapsedTime();
//
//
//        int nrcicluriSample = 0;
//        int nrcicluriSpecimen = 0;
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("init");
//        telemetryA.update();
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//
//            switch (status)
//            {
//                case START:
//                {
//                    drive.followPath(preload);
//                    status = STROBOT.PRELOAD;
//                    break;
//                }
//
//                case PRELOAD:
//                {
//                    if(!drive.isBusy())
//                    {
//                        outtake.CS = outtakeController.outtakeStatus.SPECIMEN_HIGH;
//                        timer_preload.reset();
//                        status = STROBOT.PRELOAD_SCORE;
//                    }
//                    break;
//                }
//
//                case PRELOAD_SCORE:
//                {
//                    if(timer_preload.seconds()>1)
//                    {
//                        fourbar.CS = fourbarController.fourbarStatus.SCORE_SPECIMEN_HIGH;
//                        timer_wait.reset();
//                        status = STROBOT.PRELOAD_CLAW;
//                    }
//                    break;
//                }
//
//                case PRELOAD_CLAW:
//                {
//                    if(timer_wait.seconds()>0.5)
//                    {
//                        claw.CS = clawController.clawStatus.OPENED;
//                        timer_pleaca.reset();
//                        status = STROBOT.GO_TO_COLLECT;
//                    }
//                    break;
//                }
//
//                case GO_TO_COLLECT:
//                {
//                    if(timer_pleaca.seconds()>0.5)
//                    {
//                        drive.followPath(collectSample);
//                        timer_down.reset();
//                        status = STROBOT.GO_TO_OUTTAKE_DOWN;
//                    }
//                    break;
//                }
//
//                case GO_TO_OUTTAKE_DOWN:
//                {
//                    if(timer_down.seconds()>0.25)
//                    {
//                        outtake.CS = outtakeController.outtakeStatus.DRIVE;
//                        status = STROBOT.COLLECT_EXTEND;
//                    }
//                    break;
//                }
//
//                case COLLECT_EXTEND:
//                {
//                    if(!drive.isBusy())
//                    {
//                        switch (nrcicluriSample)
//                        {
//                            case 0:
//                            {
//                                extendo.CS = extendoController.extendoStatus.SPECIMEN1;
//                                timer_wait.reset();
//                                status = STROBOT.COLLECT_DROP;
//                                break;
//                            }
//
//                            case 1:
//                            {
//                                extendo.CS = extendoController.extendoStatus.SPECIMEN2;
//                                timer_wait.reset();
//                                status = STROBOT.COLLECT_DROP;
//                                break;
//                            }
//
//                            case 2:
//                            {
//                                extendo.CS = extendoController.extendoStatus.SPECIMEN3;
//                                timer_wait.reset();
//                                status = STROBOT.COLLECT_DROP;
//                                break;
//                            }
//                        }
//                    }
//                    break;
//                }
//
//                /**
//                 * DE REFACUT CHESTIA ASTA
//                 */
//
//                case COLLECT_DROP:
//                {
//                    if(r.extendo.getCurrentPosition()>800)
//                    {
//                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//                        r.collect.setPower(0.5);
//                        status = STROBOT.COLLECT_VERIF;
//                    }
//                    break;
//                }
//
//                case COLLECT_VERIF:
//                {
//                    if(r.color_sensor.getDistance(DistanceUnit.MM)<10)
//                    {
//                        status = STROBOT.SAMPLE_DROP;
//                    }
//                    break;
//                }
//
//                case SAMPLE_DROP:
//                {
//                    switch (nrcicluriSample)
//                    {
//                        case 0:
//                        {
//                            drive.followPath(dropSample1);
//                            status = STROBOT.SPIT;
//                            break;
//                        }
//                        case 1:
//                        {
//                            drive.followPath(dropSample2);
//                            status = STROBOT.SPIT;
//                            break;
//                        }
//                        case 2:
//                        {
//                            drive.followPath(dropSample3);
//                            status = STROBOT.SPIT;
//                            break;
//                        }
//                    }
//                    break;
//                }
//
//                case SPIT:
//                {
//                    if(!drive.isBusy())
//                    {
//                        r.collect.setPower(-0.5);
//                        status = STROBOT.SAMPLE_VERIF;
//                    }
//                    break;
//                }
//
//                case SAMPLE_VERIF:
//                {
//                    if(r.color_sensor.getDistance(DistanceUnit.MM)>20)
//                    {
//                        r.collect.setPower(0);
//                        status = STROBOT.GO_TO_POSE;
//                    }
//                    break;
//                }
//
//                case GO_TO_POSE:
//                {
//                    switch (nrcicluriSample)
//                    {
//                        case 0:
//                        {
//                            drive.followPath(sample2);
//                            nrcicluriSample++;
//                            status = STROBOT.COLLECT_EXTEND;
//                            break;
//                        }
//                        case 1:
//                        {
//                            drive.followPath(sample3);
//                            nrcicluriSample++;
//                            status = STROBOT.COLLECT_EXTEND;
//                            break;
//                        }
//
//                        case 2:
//                        {
//                            status = STROBOT.GO_TO_COLLECT_SPECIMEN;
//                            break;
//                        }
//                    }
//                    break;
//                }
//
//                /*
//
//                case COLLECT_EXTEND:
//                {
//                    if(!drive.isBusy())
//                    {
//                        extendo.CS = extendoController.extendoStatus.EXTENDED;
//                        timer_wait.reset();
//                        status = STROBOT.COLLECT_DROP;
//                    }
//                    break;
//                }
//
//                case COLLECT_DROP:
//                {
//                    if(r.extendo.getCurrentPosition()>800)
//                    {
//                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//                        r.collect.setPower(0.5);
//                        status = STROBOT.COLLECT_VERIF;
//                    }
//                    break;
//                }
//
//                case COLLECT_VERIF:
//                {
//                    if(r.color_sensor.getDistance(DistanceUnit.MM)<10)
//                    {
//                        transfer.CS = transferController.transferStatus.TRANSFER_BEGIN;
//                        status = STROBOT.SAMPLE_DROP;
//                    }
//                    break;
//                }
//
//
//                case SAMPLE_DROP:
//                {
//                    switch (nrcicluriSample)
//                    {
//                        case 0:
//                        {
//                            nrcicluriSample++;
//                            drive.followPath(dropSample1);
//                            status = STROBOT.FOURBAR;
//                            break;
//                        }
//                        case 1:
//                        {
//                            nrcicluriSample++;
//                            drive.followPath(dropSample2);
//                            status = STROBOT.FOURBAR;
//                            break;
//                        }
//                        case 2:
//                        {
//                            nrcicluriSample++;
//                            drive.followPath(dropSample3);
//                            status = STROBOT.FOURBAR;
//                            break;
//                        }
//                    }
//                    break;
//                }
//
//                case FOURBAR:
//                {
//                    if(!drive.isBusy() && transfer.CS == transferController.transferStatus.TRANSFER_DONE)
//                    {
//                        fourbar.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
//                        timer_fourbar.reset();
//                        status = STROBOT.CLAW_DROP;
//                    }
//                    break;
//                }
//
//                case CLAW_DROP:
//                {
//                    if(timer_fourbar.seconds()>0.5)
//                    {
//                        claw.CS = clawController.clawStatus.OPENED;
//                        status = STROBOT.OUTTAKE_DOWN;
//                    }
//                    break;
//                }
//
//                case OUTTAKE_DOWN:
//                {
//                    outtake.CS = outtakeController.outtakeStatus.DRIVE;
//                    status = STROBOT.COLLECT_SAMPLE;
//                    break;
//                }
//
//                 */
//
//                case GO_TO_COLLECT_SPECIMEN:
//                {
//                    drive.followPath(collectFirstSpecimen);
//                    outtake.CS = outtakeController.outtakeStatus.COLLECT_SPECIMEN;
//                    status = STROBOT.PREPARE_COLLECT;
//                    break;
//                }
//
//                case PREPARE_COLLECT:
//                {
//                    if(!drive.isBusy())
//                    {
//                        status = STROBOT.VERIF_CLAW;
//                    }
//                    break;
//                }
//
//                case VERIF_CLAW:
//                {
//                    if(r.claw_sensor.getState())
//                    {
//                        claw.CS = clawController.clawStatus.CLOSED;
//                        timer_claw.reset();
//                        status = STROBOT.FOURBAR_UP;
//                    }
//                    break;
//                }
//
//                case FOURBAR_UP:
//                {
//                    if(timer_claw.seconds()>0.2)
//                    {
//                        fourbar.CS = fourbarController.fourbarStatus.SPECIMEN_UP;
//                        timer_fourbar.reset();
//                        status = STROBOT.SCORE_SPECIMEN;
//                    }
//                    break;
//                }
//
//
//
//
//                case SCORE_SPECIMEN:
//                {
//                    switch (nrcicluriSpecimen)
//                    {
//                        case 0:
//                        {
//                            drive.followPath(scoreSpecimen1);
//                            status = STROBOT.SCORE;
//                            break;
//                        }
//                        case 1:
//                        {
//                            drive.followPath(scoreSpecimen2);
//                            status = STROBOT.SCORE;
//                            break;
//                        }
//                        case 2:
//                        {
//                            drive.followPath(scoreSpecimen3);
//                            status = STROBOT.SCORE;
//                            break;
//                        }
//                        case 3:
//                        {
//                            drive.followPath(scoreSpecimen4);
//                            status = STROBOT.SCORE;
//                            break;
//                        }
//                        case 4:
//                        {
//                            drive.followPath(scoreSpecimen5);
//                            status = STROBOT.SCORE;
//                            break;
//                        }
//                    }
//                    break;
//                }
//
//                case SCORE:
//                {
//                    if(!drive.isBusy())
//                    {
//                        outtake.CS = outtakeController.outtakeStatus.SPECIMEN_HIGH;
//                        timer_specimen.reset();
//                        status = STROBOT.OUTTAKE_SCORE;
//                    }
//                    break;
//                }
//
//                case OUTTAKE_SCORE:
//                {
//                    if(timer_specimen.seconds()>1)
//                    {
//                        fourbar.CS = fourbarController.fourbarStatus.SCORE_SPECIMEN_HIGH;
//                        timer_wait.reset();
//                        status = STROBOT.SPECIMEN_CLAW;
//                    }
//                    break;
//                }
//
//                case SPECIMEN_CLAW:
//                {
//                    if(timer_wait.seconds()>0.5)
//                    {
//                        claw.CS = clawController.clawStatus.OPENED;
//                        timer_pleaca.reset();
//                        status = STROBOT.GO_TO_SPECIMEN_COLLECT;
//                    }
//                    break;
//                }
//
//                case GO_TO_SPECIMEN_COLLECT:
//                {
//                    if(timer_pleaca.seconds()>0.5)
//                    {
//                        switch (nrcicluriSpecimen)
//                        {
//                            case 0:
//                            {
//                                nrcicluriSpecimen++;
//                                drive.followPath(collectSpecimen1);
//                                timer_down.reset();
//                                status = STROBOT.GET_OUTTAKE_DOWN;
//                                break;
//                            }
//                            case 1:
//                            {
//                                nrcicluriSpecimen++;
//                                drive.followPath(collectSpecimen2);
//                                timer_down.reset();
//                                status = STROBOT.GET_OUTTAKE_DOWN;
//                                break;
//                            }
//                            case 2:
//                            {
//                                nrcicluriSpecimen++;
//                                drive.followPath(collectSpecimen3);
//                                timer_down.reset();
//                                status = STROBOT.GET_OUTTAKE_DOWN;
//                                break;
//                            }
//                            case 3:
//                            {
//                                nrcicluriSpecimen++;
//                                drive.followPath(collectSpecimen4);
//                                timer_down.reset();
//                                status = STROBOT.GET_OUTTAKE_DOWN;
//                                break;
//                            }
//                            case 4:
//                            {
//                                nrcicluriSpecimen++;
//                                drive.followPath(collectSpecimen5);
//                                timer_down.reset();
//                                status = STROBOT.GET_OUTTAKE_DOWN2;
//                                break;
//                            }
//                        }
//                    }
//                    break;
//                }
//
//
//                case GET_OUTTAKE_DOWN:
//                {
//                    if(timer_down.seconds()>0.25)
//                    {
//                        outtake.CS = outtakeController.outtakeStatus.DRIVE;
//                        status = STROBOT.PREPARE_COLLECT;
//                    }
//                    break;
//                }
//
//                case GET_OUTTAKE_DOWN2:
//                {
//                    if(timer_down.seconds()>0.25)
//                    {
//                        outtake.CS = outtakeController.outtakeStatus.DRIVE;
//                        status = STROBOT.NOTHING;
//
//                    }
//                    break;
//                }
//            }
//
//            claw.update(r);
//            clawAngle.update(r);
//            collectAngle.update(r);
//            extendo.update(r,0,0.5,voltage);
//            fourbar.update(r);
//            lift.update(r,0,voltage);
//            outtake.update(r,claw,lift,fourbar,clawAngle,transfer);
//            transfer.update(r,claw,collectAngle,outtake,extendo);
//
//
//            telemetryA.addData("caz", status);
//            telemetryA.addData("time",timer);
//            telemetryA.addData("x", drive.getPose().getX());
//            telemetryA.addData("y", drive.getPose().getY());
//
//
//            telemetryA.update();
//            drive.update();
//            drive.telemetryDebug(telemetryA);
//
//
//
//        }
//    }
//}
