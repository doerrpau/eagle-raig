import java.io.*;
import java.util.*;
import java.lang.*;

public class PSDUtility
{

    public static void main(String args[]) throws IOException
    {
        // Get IMU singletons
        IMU imu_lsm = IMU.getSingleton(IMU.IMUType.LSM330); 
        IMU imu_mpu = IMU.getSingleton(IMU.IMUType.MPU6050);

        // Calibrate IMUs
        imu_lsm.calibrate(1000);
        imu_mpu.calibrate(1000);

        // Calculate PSDs
        imu_lsm.calculatePSD(1000);
        imu_mpu.calculatePSD(1000);
       
        // Read ARWs
        for (int i = 0; i < imu_lsm.getNumSensors(); i++) {
            System.out.println("LSMZ" + i + ":\t" + IMU.toDegrees(IMU.toRRW(imu_lsm.getPSDs()[i][2])));
        }
        for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            System.out.println("MPUZ" + i + ":\t" + IMU.toDegrees(IMU.toRRW(imu_mpu.getPSDs()[i][2])));
        }
        System.out.println("LSMFUSEDZ" + ":\t" + IMU.toDegrees(IMU.toRRW(imu_lsm.getFusedPSDs()[2])));
        System.out.println("MPUFUSEDZ" + ":\t" + IMU.toDegrees(IMU.toRRW(imu_mpu.getFusedPSDs()[2])));

        // Done, close threads and exit
        System.exit(0);
    }
}
