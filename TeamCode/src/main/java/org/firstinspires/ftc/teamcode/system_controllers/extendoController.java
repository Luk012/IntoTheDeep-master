package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class extendoController {
    public enum extendoStatus {
        INITIALIZE,
        RETRACTED,
        EXTENDED,

        EXTENDED_AUTO,
        SHORT,
        TRANSFER,
        SPECIMEN1,
        SPECIMEN2,
        SPECIMEN3
    }

    // PID constants for extension
    public static double Kp_extend = 0.00008;
    public static double Ki_extend = 0;
    public static double Kd_extend = 0;

    public static double Kp_retract = 0.001; //0.01
    public static double Ki_retract = 0; //0
    public static double Kd_retract = 0; //0

    SimplePIDController extendoPIDExtend;
    SimplePIDController extendoPIDRetract;

    public static double maxSpeed = 1;

    public extendoStatus CS = INITIALIZE, PS = INITIALIZE;

    globals globals = new globals();

    public static double retracted = -5;
    public static double extended = /*30000*/ 25000;
    public static double drive = 800;
    public static double transfer = -20;
    public static double short_pose = 10000;
    public static double specimen1 = 200;
    public static double specimen2 = 200;
    public static double specimen3 = 200;
    public static double extended_auto[]= {27500, 29500, 25000};

    public static int extendo_i = 0;
    public static double i_multiplication = 50;

    public SimplePIDController activePID;

    public static double extend_multiply_index = 0;

    public extendoController() {
        extendoPIDExtend = new SimplePIDController(Kp_extend, Ki_extend, Kd_extend);
        extendoPIDRetract = new SimplePIDController(Kp_retract, Ki_retract, Kd_retract);

        extendoPIDExtend.targetValue = retracted;
        extendoPIDExtend.maxOutput = maxSpeed;

        extendoPIDRetract.targetValue = retracted;
        extendoPIDRetract.maxOutput = maxSpeed;
    }

    public void update(robotMap r, int position, double powerCap, double voltage) {
        switch (CS) {
            case INITIALIZE:
                activePID = extendoPIDRetract;
                break;
            case EXTENDED: // Define your conditions
                activePID = extendoPIDExtend;
                break;
            case RETRACTED:
                activePID = extendoPIDRetract;
                break;
            case SHORT:
                activePID = extendoPIDExtend;
                break;
            case TRANSFER:
                activePID = extendoPIDRetract;
                break;
            case EXTENDED_AUTO:
                activePID = extendoPIDExtend;
                break;
            default:
                activePID = extendoPIDRetract;
                break;
        }

        double powerColectare = activePID.update(position);
        powerColectare = Math.max(-1,Math.min(powerColectare,1));

        if (activePID.targetValue <= 0 && r.extendo.getCurrentPosition() <=  150 && CS == RETRACTED) {
            r.extendo.setPower(0);
        } else
        if(activePID.targetValue > 0 || r.extendo.getCurrentPosition() > 150)
        {
            r.extendo.setPower(powerColectare);
        }

        if (CS == RETRACTED) {
            activePID.targetValue = retracted;
        }

        if (CS != PS || CS == EXTENDED || CS == RETRACTED || CS == SHORT || CS == TRANSFER) {
            switch (CS) {
                case INITIALIZE: {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED: {
                    activePID.targetValue = extended;
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED_AUTO: {
                    activePID.targetValue = extended_auto[org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i];
                    activePID.maxOutput = 1;
                    break;
                }


                case RETRACTED: {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case SHORT: {
                    activePID.targetValue = short_pose;
                    activePID.maxOutput = 1;
                    break;
                }

                case TRANSFER: {
                    activePID.targetValue = transfer;
                    activePID.maxOutput = 1;
                    break;
                }

                case SPECIMEN1:
                {
                    activePID.targetValue = specimen1;
                    activePID.maxOutput = 0.5;
                    break;
                }

                case SPECIMEN2:
                {
                    activePID.targetValue = specimen2;
                    activePID.maxOutput = 0.5;
                    break;
                }

                case SPECIMEN3:
                {
                    activePID.targetValue = specimen3;
                    activePID.maxOutput = 0.5;
                    break;
                }


            }
        }
        PS = CS;
    }

}