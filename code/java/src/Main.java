import java.io.*;
import java.util.*;
import java.lang.*;

public class Main
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

        // Start data processing
        imu_lsm.start();
        imu_mpu.start();
       
        long start_time = System.currentTimeMillis();

        while (imu_mpu.getSensorHeadings()[0][2] > -3.14 && imu_mpu.getSensorHeadings()[0][2] < 3.14) {
            //for (int i = 0; i < imu_lsm.getNumSensors(); i++) {
            //    System.out.println("LSM" + i + ":\t" + imu_lsm.getSensorHeadings()[i][2]);
            //}
            //for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            //    System.out.println("MPU" + i + ":\t" + imu_mpu.getSensorHeadings()[i][2]);
            //}
            System.out.println("MPU0:\t" + imu_mpu.getSensorHeadings()[0][2]);
            System.out.println("LSM1:\t" + imu_lsm.getSensorHeadings()[1][2]);
            System.out.println("Fused:\t" + imu_mpu.getFusedHeadings()[2]);
            try {
                Thread.sleep(200);
            } 
            catch (Exception e) { return; }
        }

        System.out.println("It took " + ((System.currentTimeMillis() - start_time)/1000) + 
                           " seconds for mpu random walk to exceed Pi radians");

        // Stop other threads
        imu_lsm.exit();
        imu_mpu.exit();
        RAIGDriver.getSingleton().close();
    }
}
