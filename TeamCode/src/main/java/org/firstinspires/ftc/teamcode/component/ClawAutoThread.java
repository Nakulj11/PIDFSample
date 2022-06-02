package org.firstinspires.ftc.teamcode.component;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.core.Moby;

public class ClawAutoThread extends Thread{

    private Level position;

    public ClawAutoThread(Level position){
        this.position = position;
    }

    @Override
    public void run(){
        switch (position) {
            case TOP:
                for(int i=0;i<1;i++){
                    Moby.arm.moveArm(Arm.TurnValueAuto.TOP);
                    while(Moby.arm.isBusy()){

                    }
                    Moby.arm.stopArm();
                }




                break;
            case BOTTOM:
                for(int i=0;i<1;i++){
                    Moby.arm.moveArm(Arm.TurnValueAuto.BOTTOM);
                    while(Moby.arm.isBusy()){

                    }
                    Moby.arm.stopArm();
                }


                break;
            case MID:
                for(int i=0;i<1;i++){
                    Moby.arm.moveArm(Arm.TurnValueAuto.MID);
                    while(Moby.arm.isBusy()){

                    }
                    Moby.arm.stopArm();
                }

                break;
            case GROUND:
                for(int i=0;i<1;i++){
                    Moby.arm.moveArm(Arm.TurnValueAuto.GROUND);
                    while(Moby.arm.isBusy()){

                    }
                    Moby.arm.stopArm();
                }

                break;

        }
    }



    public enum Level{
        TOP,
        MID,
        BOTTOM,
        GROUND
    }
}