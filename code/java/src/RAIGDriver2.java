import java.io.*;
import java.util.*;
import java.nio.*;
import java.lang.*;
import jssc.*;
import java.util.Enumeration;


public class RAIGDriver2 implements SerialPortEventListener 
{
    private static RAIGDriver2 singleton = null;

	SerialPort serialPort;

    // Queues for decoded IMU data
    public volatile LinkedList<IMUSamples> lsm_data = new LinkedList<IMUSamples>();
    public volatile LinkedList<IMUSamples> mpu_data = new LinkedList<IMUSamples>();

    // Rate, acceleration, and temperature data from a single sensor
    // at a single point in time
    public class IMUSample
    {
        // Unprocessed sensor data
        public short rateX;
        public short rateY;
        public short rateZ;
        public short accelX;
        public short accelY;
        public short accelZ;
        public short temp;
        // Sensor ID
        // Guaranteed to be from 0 to num sensors-1 
        public byte id;
    };

    // Several IMU samples from the same point in time
    public class IMUSamples
    {
        public long timestamp;
        public Vector<IMUSample> samples;
    };

    public static RAIGDriver2 getSingleton()
    {
        if (singleton == null) {
            singleton = new RAIGDriver2();
        }
        return singleton;
    }

	protected RAIGDriver2() 
    {
	    serialPort = new SerialPort("/dev/ttyUSB0");

		try {
			serialPort.openPort();
            serialPort.setParams(115200, 
                                 SerialPort.DATABITS_8,
                                 SerialPort.STOPBITS_1,
                                 SerialPort.PARITY_NONE);

            int mask = SerialPort.MASK_RXCHAR;
            serialPort.setEventsMask(mask);
            serialPort.addEventListener(this);

            // Wait for port to be opened
            Thread.sleep(8000);
		} catch (Exception e) {
			System.err.println(e.toString());
		}
	}

	/**
	 * This should be called when you stop using the port.
	 * This will prevent port locking on platforms like Linux.
	 */
	public synchronized void close() {
        try {
		    if (serialPort != null) {
			    serialPort.removeEventListener();
			    serialPort.closePort();
		    }
        } catch (Exception e) {
            System.err.println(e.toString());
        }
	}

	/**
	 * Handle an event on the serial port. Read the data and print it.
	 */
	public synchronized void serialEvent(SerialPortEvent oEvent) {
		if (oEvent.isRXCHAR()) {
            demarshall();	
		}
	}

    // Demarshall the RAIG messages and put on message queue
    private synchronized void demarshall()
    {
        try {
            IMUSamples newLsms = new IMUSamples();
            newLsms.samples = new Vector<IMUSample>();
            IMUSamples newMpus = new IMUSamples();
            newMpus.samples = new Vector<IMUSample>();

            byte checksum = 0;

		    while (true) {
                char curByte = (char)serialPort.readBytes(1)[0];
                // State machine processes messages
                switch (curByte) {
                    case 'L':
                        IMUSample newLsm = new IMUSample();
                        // Get sensor ID
                        newLsm.id = (byte)serialPort.readBytes(1)[0];
                        // Get rate data
                        newLsm.rateX = (short)(newLsm.rateX | (serialPort.readBytes(1)[0] & 0xFF));
                        newLsm.rateX = (short)(newLsm.rateX | ((serialPort.readBytes(1)[0] & 0xFF) << 8));
                        newLsm.rateY = (short)(newLsm.rateY | (serialPort.readBytes(1)[0] & 0xFF));
                        newLsm.rateY = (short)(newLsm.rateY | ((serialPort.readBytes(1)[0] << 8) & 0xFF00));
                        newLsm.rateZ = (short)(newLsm.rateZ | (serialPort.readBytes(1)[0] & 0xFF));
                        newLsm.rateZ = (short)(newLsm.rateZ | ((serialPort.readBytes(1)[0] << 8) & 0xFF00));
                        // Get acceleration data
                        newLsm.accelX = (short)(newLsm.accelX | (serialPort.readBytes(1)[0] & 0xFF));
                        newLsm.accelX = (short)(newLsm.accelX | ((serialPort.readBytes(1)[0] << 8) & 0xFF00));
                        newLsm.accelY = (short)(newLsm.accelY | (serialPort.readBytes(1)[0] & 0xFF));
                        newLsm.accelY = (short)(newLsm.accelY | ((serialPort.readBytes(1)[0] << 8) & 0xFF00));
                        newLsm.accelZ = (short)(newLsm.accelZ | (serialPort.readBytes(1)[0] & 0xFF));
                        newLsm.accelZ = (short)(newLsm.accelZ | ((serialPort.readBytes(1)[0] << 8) & 0xFF00));
                        // Get temperature data
                        newLsm.temp = (byte)serialPort.readBytes(1)[0];
                        newLsms.samples.add(newLsm);
                        break;
                    case 'M':
                        IMUSample newMpu = new IMUSample();
                        // Get sensor ID
                        newMpu.id = (byte)serialPort.readBytes(1)[0];
                        // Get rate data
                        newMpu.rateX = (short)serialPort.readBytes(1)[0];
                        newMpu.rateX = (short)((newMpu.rateX << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        newMpu.rateY = (short)serialPort.readBytes(1)[0];
                        newMpu.rateY = (short)((newMpu.rateY << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        newMpu.rateZ = (short)serialPort.readBytes(1)[0];
                        newMpu.rateZ = (short)((newMpu.rateZ << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        // Get acceleration data
                        newMpu.accelX = (short)serialPort.readBytes(1)[0];
                        newMpu.accelX = (short)((newMpu.accelX << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        newMpu.accelY = (short)serialPort.readBytes(1)[0];
                        newMpu.accelY = (short)((newMpu.accelY << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        newMpu.accelZ = (short)serialPort.readBytes(1)[0];
                        newMpu.accelZ = (short)((newMpu.accelZ << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        // Get temperature data
                        newMpu.temp = (short)serialPort.readBytes(1)[0];
                        newMpu.temp = (short)((newMpu.temp << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        newMpus.samples.add(newMpu);
                        break;
                    case 'T':
                        long time = serialPort.readBytes(1)[0];
                        time = ((time << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        time = ((time << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        time = ((time << 8) | (serialPort.readBytes(1)[0] & 0xFF));
                        serialPort.readBytes(1);
                        // TODO: Implement checksum
                        // Add to queues and exit
                        if (!newLsms.samples.isEmpty()) {
                            newLsms.timestamp = time;
                            lsm_data.add(newLsms);
                        }
                        if (!newMpus.samples.isEmpty()) {
                            newMpus.timestamp = time;
                            mpu_data.add(newMpus);
                        }
                        return;
                    default: // Error in message?
                        break;
                }
            }
		} catch (Exception e) {
		    System.err.println(e.toString());
		}

    } // demarshall()

}
