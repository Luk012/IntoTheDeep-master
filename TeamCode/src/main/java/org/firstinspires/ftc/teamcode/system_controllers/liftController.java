package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SAMPLE_HIGH;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SAMPLE_LOW;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class liftController {

    public enum liftStatus
    {
        INITIALIZE,
        DOWN,
        SAMPLE_HIGH,
        SAMPLE_LOW,
        SPECIMEN_HIGH,
        SPECIMEN_LOW,
        HANG_LV1,

    }

    public double CurrentSpeed = 0;

//    public static double PDRIVE = 0.0005;
//    public static double IDRIVE = 0;
//    public static double DDRIVE = 0;
public static double PDRIVE = 0.0002;
    public static double IDRIVE = 0;
    public static double DDRIVE = 0.002;

    public static double PDOWN = 0.0055;
    public static double IDOWN = 0;
    public static double DDOWN = 0;

    public double maxSpeedUp = 1;

    public static liftStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController LiftPIDDOWN = null;
    SimplePIDController LiftPID_DRIVE = null;
    public SimplePIDController activePID;

    public static double base = -40;
    public static double specimen_high = 22500;
    public static double specimen_low = 0;
    public static double sample_high = 46000;
    public static double sample_low = 15000;
    public static double hang = 14000;

    public int CurrentPosition = 0;
   // public static double transfer = -100;

    public liftController()
    {
        LiftPIDDOWN = new SimplePIDController(PDOWN,IDOWN,DDOWN);
        LiftPIDDOWN.targetValue = base;
        LiftPIDDOWN.maxOutput = maxSpeedUp;

        LiftPID_DRIVE = new SimplePIDController(PDRIVE,IDRIVE,DDRIVE);
        LiftPID_DRIVE.targetValue = base;
        LiftPID_DRIVE.maxOutput = maxSpeedUp;
    }


    public void disableMotor(robotMap r) {
        r.lift.setMotorDisable();
    }

    public void enableMotor(robotMap r)
    {
        r.lift.setMotorEnable();
    }

    public void update(robotMap r, int position, double voltage)
    {
        switch (CS) {
            case DOWN:
                activePID = LiftPIDDOWN;
                break;
            case SAMPLE_HIGH:
                activePID = LiftPID_DRIVE;
                break;
            case SAMPLE_LOW:
                activePID = LiftPID_DRIVE;
                break;
            case SPECIMEN_HIGH:
                activePID = LiftPID_DRIVE;
                break;
            case SPECIMEN_LOW:
                activePID = LiftPID_DRIVE;
                break;
            case HANG_LV1:
                activePID = LiftPID_DRIVE;
                break;
            default:
                activePID = LiftPIDDOWN;
                break;
        }

        CurrentPosition = position;
        double powerLift = Math.min(activePID.update(position), 1);
        powerLift =  Math.max(-1,Math.min(powerLift,1));
        CurrentSpeed=powerLift;

        if(CS == DOWN && r.collect.getCurrentPosition() > 150) r.lift.setPower(-0.9);
        else if (activePID.targetValue <= 0 && r.collect.getCurrentPosition() <=  150 && ( CS == DOWN)) {
           r.lift.setPower(0);
        } else
        if((activePID.targetValue > 0 || r.collect.getCurrentPosition() > 150) && CS != DOWN)
        {
            r.lift.setPower(powerLift);
        }

        if(CS != PS || CS == SAMPLE_LOW || CS == SAMPLE_HIGH)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    activePID.targetValue = base;
                    break;
                }

                case SPECIMEN_HIGH:
                {
                    activePID.targetValue = specimen_high;
                    break;
                }

                case DOWN:
                {
                    activePID.targetValue = base;
                    break;
                }

                case SPECIMEN_LOW:
                {
                    activePID.targetValue = specimen_low;
                    break;
                }

                case SAMPLE_HIGH:
                {
                    activePID.targetValue = sample_high;
                    break;
                }

                case SAMPLE_LOW:
                {
                    activePID.targetValue = sample_low;
                    break;
                }

                case HANG_LV1:
                {
                    activePID.targetValue = hang;
                    break;
                }
            }
        }

        PS = CS;
    }

}