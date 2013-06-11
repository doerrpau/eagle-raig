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

        // Read 0-offsets
        for (int i = 0; i < imu_lsm.getNumSensors(); i++) {
            System.out.println("LSM" + i + ":\t" + imu_lsm.getOffsets()[i][0]
                                          + "\t" + imu_lsm.getOffsets()[i][1]
                                          + "\t" + imu_lsm.getOffsets()[i][2]);
        }
        for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            System.out.println("MPU" + i + ":\t" + imu_mpu.getOffsets()[i][0]
                                          + "\t" + imu_mpu.getOffsets()[i][1]
                                          + "\t" + imu_mpu.getOffsets()[i][2]);
        }

        // Calculate PSDs
        imu_lsm.calculatePSD(1000);
        imu_mpu.calculatePSD(1000);
       
        // Read ARWs
        for (int i = 0; i < imu_lsm.getNumSensors(); i++) {
            System.out.println("LSM" + i + ":\t" + imu_lsm.getRRWs()[i][0]
                                          + "\t" + imu_lsm.getRRWs()[i][1]
                                          + "\t" + imu_lsm.getRRWs()[i][2]);
        }
        for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            System.out.println("MPU" + i + ":\t" + imu_mpu.getRRWs()[i][0]
                                          + "\t" + imu_mpu.getRRWs()[i][1]
                                          + "\t" + imu_mpu.getRRWs()[i][2]);
        }
    }
}
