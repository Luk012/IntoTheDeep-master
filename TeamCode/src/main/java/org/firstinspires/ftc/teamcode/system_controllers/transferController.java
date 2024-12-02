package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_BEGIN;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_OUTAKE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_SPIT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class transferController {

    public enum transferStatus
    {
        INITIALIZE,
        TRANSFER_BEGIN,
        TRANSFER_EXTENDO,
        TRANSFER_SPIT,
        TRANSFER_CLAW,
        TRANSFER_OUTAKE,
        TRANSFER_DONE
    }

    public transferController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    ElapsedTime collectAngle_timer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime outtake_timer = new ElapsedTime();

    globals globals = new globals();

public static double extendo_limit = 0.4;
    public static transferStatus CS = INITIALIZE, PS = INITIALIZE;

    public void update(robotMap r, clawController claw, collectAngleController collectAngle, outtakeController outtake, extendoController extendo)
    {
        if(CS != PS || CS == INITIALIZE || CS == TRANSFER_BEGIN || CS == TRANSFER_EXTENDO || CS == TRANSFER_SPIT || CS == TRANSFER_CLAW || CS == TRANSFER_OUTAKE || CS == TRANSFER_DONE)
        {
            switch (CS)
            {
                case TRANSFER_BEGIN:
                {
                    globals.is_intransfer = true;
                    collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                    outtake.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                    r.collect.setPower(-1);
                    collectAngle_timer.reset();
                    CS = TRANSFER_EXTENDO;
                    break;
                }

                case TRANSFER_EXTENDO:
                {
                    if(collectAngle_timer.seconds()>extendo_limit)
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        CS = TRANSFER_SPIT;
                    }
                    break;
                }

                case TRANSFER_SPIT:
                {
                    if(r.extendo.getCurrentPosition() < 250)
                    {
                        r.collect.setPower(1);
                        claw_timer.reset();
                        CS = TRANSFER_CLAW;
                    }
                    break;
                }

                case TRANSFER_CLAW:
                {
                    if(!r.cuva_sensor.getState() || claw_timer.seconds() > 0.5)
                    {
                        claw.CS = clawController.clawStatus.CLOSED;
                        outtake_timer.reset();
                        globals.is_intransfer = false;
                            r.collect.setPower(0);
                        CS = TRANSFER_DONE;
                    }
                    break;
                }

                case TRANSFER_OUTAKE:
                {
                    if(outtake_timer.seconds() >0.5)
                    {
                        outtake.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                        globals.is_intransfer = false;
                        CS = TRANSFER_DONE;
                    }
                    break;
                }
            }
        }

        PS = CS;

    }

}
