import java.io.*;
import java.util.*;
import java.lang.*;

// This utility records gyro output v internal temperature sensor reading for all gyroscopes
// and dumps the data to csv
public class TempUtil
{

    public static void main(String args[]) throws IOException
    {
        final int NUM_SENSORS = 4;

        PrintWriter writer[] = new PrintWriter[NUM_SENSORS];
        for (int i = 0; i < NUM_SENSORS; i++) {
            writer[i] = new PrintWriter("temp_data" + i + ".csv", "UTF-8");
        }
        RAIGDriver driver = RAIGDriver.getSingleton();
        
        // Print Header
        for (int i = 0; i < NUM_SENSORS; i++) {
            writer[i].println("MPU" + i + 
                           "X,MPU" + i + 
                           "Y,MPU" + i + 
                           "Z,temp,time");
        }
        
        while (true) {
            if (!driver.mpu_data.isEmpty()) {
                long time = driver.mpu_data.getFirst().timestamp;
                for (int i = 0; i < driver.mpu_data.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample mpu_samp = driver.mpu_data.getFirst().samples.get(i);
                    
                    writer[mpu_samp.id].print(mpu_samp.rate[0] + ",");
                    writer[mpu_samp.id].print(mpu_samp.rate[1] + ",");
                    writer[mpu_samp.id].print(mpu_samp.rate[2] + ",");
                    writer[mpu_samp.id].print(mpu_samp.temp + ",");
                    writer[mpu_samp.id].println(time);
                }
                driver.mpu_data.clear();
            }
            try {
                Thread.sleep(2);
            } 
            catch (Exception e) { return; }
        }
    }
}
