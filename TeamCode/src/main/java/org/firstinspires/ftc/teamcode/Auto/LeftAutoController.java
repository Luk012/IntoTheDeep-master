package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;


public class LeftAutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        SAMPLE_START,
        SAMPLE_FOURBAR,

        COLLECT_SAMPLE,
        COLLECT_SAMPLE_ANGLE,
        COLLECT_SAMPLE_DONE,

        COLLECT_SAMPLE_EXTEND_FULL,


        OUTTAKE_DOWN,
        OPEN_CLAW,
        OPEN_CLAW_DONE,
        OUTTAKE_DOWN_SYSTEMS,
        OUTTAKE_DOWN_FOURBAR,
        OUTTAKE_DOWN_DONE,

        TRANSFER_BEGIN,
        TRANSFER_EXTENDO,
        TRANSFER_SPIT,
        TRANSFER_CLAW,
        TRANSFER_OUTAKE,
        TRANSFER_DONE,
        SAMPLE_DONE,
        HANG_LV1

    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    public static autoControllerStatus CurrentStatus2 = autoControllerStatus.NOTHING, PreviousStatus2 = autoControllerStatus.NOTHING;


    ElapsedTime preloadTimer = new ElapsedTime();
    ElapsedTime extendoTimer = new ElapsedTime();
    ElapsedTime fourbarTimer = new ElapsedTime();


    ElapsedTime clawTimer = new ElapsedTime();
    public void update(robotMap r, clawAngleController clawAngleController, clawController clawController, collectAngleController collectAngleController, extendoController extendoController, fourbarController fourbarController, liftController liftController )
    {

        switch (CurrentStatus)
        {

            case SAMPLE_START:
            {
                liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SAMPLE_HIGH;
                preloadTimer.reset();
                CurrentStatus = autoControllerStatus.SAMPLE_FOURBAR;
                break;
            }

            case SAMPLE_FOURBAR:
            {
                if(preloadTimer.seconds() > 0.5)
                {
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SAMPLE;
                    clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SAMPlE_SCORE;
                    CurrentStatus = autoControllerStatus.SAMPLE_DONE;
                }
                break;
            }

            case COLLECT_SAMPLE:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
                CurrentStatus = autoControllerStatus.COLLECT_SAMPLE_ANGLE;
                break;
            }

            case COLLECT_SAMPLE_ANGLE:
            {
                if(r.extendo.getCurrentPosition() > 750)
                {
                    collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
                    r.collect.setPower(-0.7);
                    CurrentStatus = autoControllerStatus.COLLECT_SAMPLE_DONE;
                }break;
            }

            case COLLECT_SAMPLE_EXTEND_FULL:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AUTO;
                    CurrentStatus = autoControllerStatus.COLLECT_SAMPLE_DONE;
                    break;
            }

            case TRANSFER_BEGIN:
            {
                collectAngleController.CS = DRIVE;
                extendoTimer.reset();
                CurrentStatus = autoControllerStatus.TRANSFER_EXTENDO;
                break;
            }

            case TRANSFER_EXTENDO:
            {
                if(extendoTimer.seconds() > 0.4)
                {
                    extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
                    CurrentStatus = autoControllerStatus.TRANSFER_SPIT;
                }
                break;
            }

            case TRANSFER_SPIT:
            {
                if(r.extendo.getCurrentPosition() <= 250)
                {
                    r.collect.setPower(1);
                    clawTimer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_CLAW;
                }
                break;
            }

            case TRANSFER_CLAW:
            {
                if(!r.cuva_sensor.getState() || clawTimer.seconds() > 0.5)
                {
                    r.collect.setPower(0);
                    org.firstinspires.ftc.teamcode.system_controllers.clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.CLOSED;
                    CurrentStatus = autoControllerStatus.TRANSFER_DONE;
                }
                break;
            }

        }

       //todo switch between cs2 and cs1

        switch (CurrentStatus2)
        {

            case OUTTAKE_DOWN:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
                CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_SYSTEMS;
                //outtakeTimer.reset();
                break;
            }

            case OUTTAKE_DOWN_SYSTEMS:
            {
                if(r.collect.getCurrentPosition() < 500)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                    clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.TRANSFER;
                    fourbarTimer.reset();
                    CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_FOURBAR;
                }
                break;
            }

            case OUTTAKE_DOWN_FOURBAR:
            {
                if(fourbarTimer.seconds() > 0.2)
                {
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.TRANSFER;
                    CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_DONE;
                }
                break;
            }

            case OPEN_CLAW:
            {
                clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                    CurrentStatus2 = autoControllerStatus.OPEN_CLAW_DONE;
                    break;
            }
            case HANG_LV1:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.HANG_LV1;
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.HANG_LV1;
                clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.HANG_LV1;
                CurrentStatus2 = autoControllerStatus.NOTHING;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
        PreviousStatus2 = CurrentStatus2;
    }
}
