package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class hood {
    private Servo hood;
    public static double open = 0.34;
    public static double close = 0.2;
    public static double hoodpos ;

    public void init(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood");


    }

    public double autoshoot(double x){return MathFunctions.clamp(
            -(8.88777e-8) * Math.pow(x, 4)
                    + 0.0000266722 * Math.pow(x, 3)
                    - 0.00269661 * Math.pow(x, 2)
                    + 0.0925372 * x
                    + 0.123307,
            0.11,
            0.92
    );
    }
    public void setPosition(double position){hood.setPosition(position);
    }


}
