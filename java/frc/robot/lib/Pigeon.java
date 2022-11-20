package frc.robot.lib;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Ports;


public class Pigeon {

    private static Pigeon mInstance;

    public static Pigeon getInstance() {
        if (mInstance == null) {
            mInstance = new Pigeon(Ports.PIGEON2);
        }
        return mInstance;
    }

    // Actual pigeon object
    private final Pigeon2 mGyro;

    // Configs

    private Pigeon(int port) {        
        mGyro = new Pigeon2(port, "canivore1");
        mGyro.configFactoryDefault();
    }

}