package org.firstinspires.ftc.teamcode.TestOpModes.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Generals.Enums;
import org.firstinspires.ftc.teamcode.Systems.Subsystems.Other.Drone;

@TeleOp(group = "test")
public class DroneTest extends LinearOpMode implements Enums.Other {
    private Drone drone;
    private GamepadEx g2;
    private Telemetry dashboardTelemetry;

    //TODO:             - daca vazut din spatele avionului (daca te deranjeaza ca nu-i important), servoul cu tensiunea se misca invers
    //TODO:               fata de joystick, pune minus aici: drone.setPower("-"g2.getLeftY());
    //TODO:             - LOAD = un feature viitor, ideea momentan ii ca ar trebui sa se roteasca gear-un pentru lansare catre varful lansatorului
    //TODO:               FIRE = tot un feature viitor, ideea momentan ii ca se invarte in sens opus la LOAD
    //TODO:               daca is invers sau daca ii prea incet, in clasa 'Drone' (o ai deschisa in tab sus), schimbi cele doua constante pentru
    //TODO:               load si pentru fire
    //TODO:             - pentru encoder, tot in 'Drone', la initializare schimbi numele din configuratie corespunzator cu cel de la portu de motor
    //TODO:               in care bagi cablu
    //TODO:
    //TODO:               daca something goes WRONG, call me bby 😘



    @Override
    public void runOpMode() throws InterruptedException {
        drone = new Drone(this.hardwareMap);
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboardTelemetry.addLine("PRESS START");
        dashboardTelemetry.update();

        waitForStart();

        dashboardTelemetry.clearAll();

        while (opModeIsActive()) {

            //pentru servo-u care da drumu la avion
            if (g2.isDown(GamepadKeys.Button.DPAD_UP)) { drone.setState(LauncherPositions.LOAD); }
            else if (g2.isDown(GamepadKeys.Button.DPAD_DOWN)) { drone.setState(LauncherPositions.FIRE); }
            else { drone.setState(LauncherPositions.PENDING); }

            //pentru servo-u cu tensiunea
            drone.setPower(g2.getLeftY());

            dashboardTelemetry.addData("encoder position: ", drone.getEncoderPosition());
            dashboardTelemetry.update();
            drone.update();
            g2.readButtons();
        }


    }
}
