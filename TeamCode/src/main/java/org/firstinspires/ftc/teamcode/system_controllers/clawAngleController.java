package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SAMPlE_SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class clawAngleController {

    public enum clawAngleStatus
    {
        INITIALIZE,
        COLLECT_SPECIMEN,
        SAMPlE_SCORE,
        SPECIMEN_SCORE,
        TRANSFER,
        SPECIMEN,
        HANG_LV1
    }

    public clawAngleController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static clawAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double collect_specimen = 0.52;
    public static double sample_score = 0.2;
    public static double specimen = 0.5;
    public static double transfer = 0.03;
    public static double specimen_score = 0.5;
    public static double hang_lv1 = 0.5;

    public void update(robotMap r)
    {
        if(PS != CS || CS == INITIALIZE || CS == COLLECT_SPECIMEN || CS == SAMPlE_SCORE || CS == SPECIMEN_SCORE || CS == TRANSFER || CS == SPECIMEN)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.clawAngle.setPosition(transfer);
                    break;
                }

                case COLLECT_SPECIMEN:
                {
                    r.clawAngle.setPosition(collect_specimen);
                    break;
                }

                case SAMPlE_SCORE:
                {
                    r.clawAngle.setPosition(sample_score);
                    break;
                }

                case SPECIMEN_SCORE:
                {
                    r.clawAngle.setPosition(specimen_score);
                    break;
                }

                case TRANSFER:
                {
                    r.clawAngle.setPosition(transfer);
                    break;
                }

                case SPECIMEN:
                {
                    r.clawAngle.setPosition(specimen);
                    break;
                }

                case HANG_LV1:
                {
                    r.clawAngle.setPosition(hang_lv1);
                    break;
                }
            }
        }

        PS = CS;
    }

}
