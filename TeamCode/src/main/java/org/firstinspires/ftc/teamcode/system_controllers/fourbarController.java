package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT_SPECIMEN_INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SAMPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class fourbarController {

    public enum fourbarStatus {
        SCORE_SPECIMEN,
        COLLECT_SPECIMEN,
        INITIALIZE,
        SAMPLE,
        SPECIMEN,
        TRANSFER,
        INTER,
        COLLECT_SPECIMEN_INTER,
        HANG_LV1
    }

    public fourbarController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static fourbarStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double score_specimen = 0.9;
    public static double collect = 0.25;
    public static double sample = 0.57;
    public static double specimen = 1;
    public static double transfer = 0.065;
    public static double inter  = 0.7;
    public static double collect_specimen_inter = 0.4;
    public static double hang_lv1 = 0.42;

    public void update(robotMap r)
    {
        if (PS != CS || CS == INITIALIZE || CS == COLLECT_SPECIMEN  || CS == SCORE_SPECIMEN  || CS == TRANSFER  || CS == SPECIMEN || CS == SAMPLE || CS == COLLECT_SPECIMEN_INTER)
        {

            switch (CS) {
                case INITIALIZE: {
                    r.fourbarLeft.setPosition(transfer);
                    r.fourbarRight.setPosition(transfer);
                    break;
                }

                case COLLECT_SPECIMEN: {
                    r.fourbarLeft.setPosition(collect);
                    r.fourbarRight.setPosition(collect);
                    break;
                }

                case SCORE_SPECIMEN: {
                    r.fourbarLeft.setPosition(score_specimen);
                    r.fourbarRight.setPosition(score_specimen);
                    break;
                }

                case SAMPLE: {
                    r.fourbarLeft.setPosition(sample);
                    r.fourbarRight.setPosition(sample);
                    break;
                }

                case SPECIMEN: {
                    r.fourbarLeft.setPosition(specimen);
                    r.fourbarRight.setPosition(specimen);
                    break;
                }

                case TRANSFER: {
                    r.fourbarLeft.setPosition(transfer);
                    r.fourbarRight.setPosition(transfer);
                    break;
                }

                case INTER:
                {
                    r.fourbarLeft.setPosition(inter);
                    r.fourbarRight.setPosition(inter);
                    break;
                }

                case COLLECT_SPECIMEN_INTER:
                {
                    r.fourbarLeft.setPosition(collect_specimen_inter);
                    r.fourbarRight.setPosition(collect_specimen_inter);
                    break;
                }

                case HANG_LV1:
                {
                    r.fourbarLeft.setPosition(hang_lv1);
                    r.fourbarRight.setPosition(hang_lv1);
                    break;
                }
            }
        }
        PS = CS;
    }

}
