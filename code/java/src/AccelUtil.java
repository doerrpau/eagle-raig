import java.io.*;
import java.util.*;
import java.lang.*;

public class AccelUtil
{


    // Print averaged accelerometer acceleration vs gyro rate
    // Used to calibrate gyroscope acceleration sensitivity
    public static void main(String args[]) throws IOException
    {
        final int NUM_SENSORS = 4;

        // Accel Axis to print
        final int axis = 2;

        // Get IMU singletons
        IMU imu_mpu = new IMU(IMU.IMUType.MPU6050);
        
        // Calibrate IMUs
        imu_mpu.calibrate(1000);

        // Start data processing
        Thread mpu_thread = new Thread(imu_mpu);
        mpu_thread.start();

        System.out.println("MPU 0 Axis,MPU 1 Axis,MPU 2 Axis,MPU 3 Axis");
        // Used to calculate average offset
        int samples[] = new int[NUM_SENSORS];
        double total_accel[] = new double[NUM_SENSORS];
        while (true) {
            for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
                System.out.format("%6.3f:",imu_mpu.imu_data[i].getAccel()[axis]);
                if (i+1 < imu_mpu.getNumSensors()) {
                    System.out.print(",");
                } else {
                    System.out.print("\n");
                }
            }
            try {
                Thread.sleep(23);
            } 
            catch (Exception e) { return; }
        }
    }
}
