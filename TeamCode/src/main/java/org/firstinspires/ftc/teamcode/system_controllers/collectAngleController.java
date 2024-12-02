package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class collectAngleController {

    public enum collectAngleStatus
    {
        INITIALIZE,
        COLLECT,
        DRIVE,
    }

    public collectAngleController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static collectAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double collect = 0.9;
    public static double drive = 0.18;

    public void update(robotMap r)
    {
         if(CS != PS || CS == INITIALIZE || CS == COLLECT || CS == DRIVE)
         {
             switch (CS)
             {
                 case INITIALIZE:
                 {
                     r.collectAngle.setPosition(drive);
                     break;
                 }

                 case COLLECT:
                 {
                     r.collectAngle.setPosition(collect);
                     break;
                 }

                 case DRIVE:
                 {
                     r.collectAngle.setPosition(drive);
                     break;
                 }
             }
         }

         PS = CS;
    }

}
