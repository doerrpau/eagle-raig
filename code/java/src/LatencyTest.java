import java.io.*;
import java.util.*;
import java.lang.*;

public class LatencyTest
{

    public static void main(String args[]) throws IOException
    {
        RAIGDriver driver = new RAIGDriver();
        long arduLastTime = 0;
        long hostLastTime = 0;
        while (true) {
            if (!driver.lsm_data.isEmpty()) {
                long arduTime = driver.lsm_data.getFirst().timestamp;
                long hostTime = System.currentTimeMillis();
                System.out.println("Message Send Latency: " + (arduTime - arduLastTime) + "\t" +
                                   "Message Received Latency: " + (hostTime - hostLastTime) + "\t" +
                                   "Queue Size: " + driver.lsm_data.size());
                arduLastTime = arduTime;
                hostLastTime = hostTime;
                driver.lsm_data.remove();

            }
            try {
                Thread.sleep(1);
            } 
            catch (Exception e) { return; }
        }
    }
}
