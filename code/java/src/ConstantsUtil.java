import java.io.*;
import java.util.*;
import java.lang.*;

public class ConstantsUtil
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
        Thread lsm_thread = new Thread(imu_lsm);
        Thread mpu_thread = new Thread(imu_mpu);
        lsm_thread.start();
        mpu_thread.start();

        // Wait for manual rotation, then read heading data again
        Scanner reader = new Scanner(System.in);
        System.out.println("Rotate 360 degrees, then press Enter");
        reader.nextLine();

        for (int i = 0; i < imu_lsm.getNumSensors(); i++) {
            System.out.println("LSM" + i + ":\t" + imu_lsm.getSensorHeadings()[i][0]
                                          + "\t" + imu_lsm.getSensorHeadings()[i][1]
                                          + "\t" + imu_lsm.getSensorHeadings()[i][2]);
        }
        for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            System.out.println("MPU" + i + ":\t" + imu_mpu.getSensorHeadings()[i][0]
                                          + "\t" + imu_mpu.getSensorHeadings()[i][1]
                                          + "\t" + imu_mpu.getSensorHeadings()[i][2]);
        }

        // Stop all threads and exit
        System.exit(0);
    }
}
