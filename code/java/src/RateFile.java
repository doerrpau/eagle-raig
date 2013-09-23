import java.io.*;
import java.util.*;
import java.lang.*;

public class RateFile
{

    public static void main(String args[]) throws IOException
    {
        // Get IMU singletons
        IMU imu_mpu = new IMU(IMU.IMUType.MPU6050);
        PrintWriter writer = new PrintWriter("rate_data.csv", "UTF-8");
        
        // Calibrate IMUs
        imu_mpu.calibrate(1000);

        // Start data processing
        Thread mpu_thread = new Thread(imu_mpu);
        mpu_thread.start();

        writer.println("MPU 0 Z Axis,MPU 1 Z Axis,MPU 2 Z Axis,MPU 3 Z Axis");
        while (true) {
            for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
                writer.format("%15.14f", imu_mpu.getSensorRates()[i][2]);
                if (i+1 < imu_mpu.getNumSensors()) {
                    writer.print(",");
                } else {
                    writer.print("\n");
                }
            }
            try {
                Thread.sleep(23);
            } 
            catch (Exception e) { return; }
        }
    }
}
