package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.CLOSED;
import static org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class clawController {
    public enum clawStatus
    {
        INITIALIZE,
        CLOSED,
        OPENED,
        COLLECT_SPECIMEN,
    }

    public clawController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static clawStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double closed = 0.57;
    public static double opened = 0.675;
    public static double collect_specimen = 0.73;

    public void update(robotMap r)
    {
        if(PS != CS || CS == INITIALIZE || CS == CLOSED || CS == OPENED || CS == COLLECT_SPECIMEN)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.claw.setPosition(opened);
                    break;
                }

                case CLOSED:
                {
                    r.claw.setPosition(closed);
                    break;
                }

                case OPENED:
                {
                    r.claw.setPosition(opened);
                    break;
                }

                case COLLECT_SPECIMEN:
                {
                    r.claw.setPosition(collect_specimen);
                    break;
                }
            }
        }

        PS = CS;
    }
}
