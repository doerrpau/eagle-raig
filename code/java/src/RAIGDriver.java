import java.io.*;
import java.util.*;
import java.lang.*;
import java.nio.*;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import gnu.io.CommPortIdentifier; 
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent; 
import gnu.io.SerialPortEventListener; 
import java.util.Enumeration;


public class RAIGDriver implements SerialPortEventListener {
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
        public byte id;
    };

    // Several IMU samples from the same point in time
    public class IMUSamples
    {
        public long timestamp;
        public Vector<IMUSample> samples;
    };

    /** The port we're normally going to use. */
	private static final String PORT_NAMES[] = { 
			"/dev/tty.usbserial-A9007UX1", // Mac OS X
			"/dev/ttyUSB0", // Linux
			"COM3", // Windows
	};
	/**
	* A BufferedReader which will be fed by a InputStreamReader 
	* converting the bytes into characters 
	* making the displayed results codepage independent
	*/
	//private BufferedReader input;
	private InputStreamReader input;
	/** Milliseconds to block while waiting for port open */
	private static final int TIME_OUT = 2000;
	/** Default bits per second for COM port. */
	private static final int DATA_RATE = 115200;

	public RAIGDriver() {
		CommPortIdentifier portId = null;
		Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

		//First, Find an instance of serial port as set in PORT_NAMES.
		while (portEnum.hasMoreElements()) {
			CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
			for (String portName : PORT_NAMES) {
				if (currPortId.getName().equals(portName)) {
					portId = currPortId;
					break;
				}
			}
		}
		if (portId == null) {
			System.out.println("Could not find COM port.");
			return;
		}

		try {
			// open serial port, and use class name for the appName.
			serialPort = (SerialPort) portId.open(this.getClass().getName(),
					TIME_OUT);

			// set port parameters
			serialPort.setSerialPortParams(DATA_RATE,
					SerialPort.DATABITS_8,
					SerialPort.STOPBITS_1,
					SerialPort.PARITY_NONE);

			// open the streams
			//input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
			input = new InputStreamReader(serialPort.getInputStream());

			// add event listeners
			serialPort.addEventListener(this);
			serialPort.notifyOnDataAvailable(true);
		} catch (Exception e) {
			System.err.println(e.toString());
		}
	}

	/**
	 * This should be called when you stop using the port.
	 * This will prevent port locking on platforms like Linux.
	 */
	public synchronized void close() {
		if (serialPort != null) {
			serialPort.removeEventListener();
			serialPort.close();
		}
	}

	/**
	 * Handle an event on the serial port. Read the data and print it.
	 */
	public synchronized void serialEvent(SerialPortEvent oEvent) {
		if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
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

		    while (input.ready()) {
                char curByte = (char)input.read();
                // State machine processes messages
                switch (curByte) {
                    case 'L':
                        IMUSample newLsm = new IMUSample();
                        // Get sensor ID
                        newLsm.id = (byte)input.read();
                        // Get rate data
                        ByteBuffer bb = ByteBuffer.allocate(2);
                        bb.order(ByteOrder.BIG_ENDIAN);
                        bb.put((byte)input.read());
                        bb.put((byte)input.read());
                        newLsm.rateX = bb.getShort(0);
                        //newLsm.rateX = (short)(newLsm.rateX | (input.read() & 0xFF));
                        //newLsm.rateX = (short)(newLsm.rateX | ((input.read() & 0xFF) << 8));
                        newLsm.rateY = (short)(newLsm.rateY | (input.read() & 0xFF));
                        newLsm.rateY = (short)(newLsm.rateY | ((input.read() << 8) & 0xFF00));
                        newLsm.rateZ = (short)(newLsm.rateZ | (input.read() & 0xFF));
                        newLsm.rateZ = (short)(newLsm.rateZ | ((input.read() << 8) & 0xFF00));
                        // Get acceleration data
                        newLsm.accelX = (short)(newLsm.accelX | (input.read() & 0xFF));
                        newLsm.accelX = (short)(newLsm.accelX | ((input.read() << 8) & 0xFF00));
                        newLsm.accelY = (short)(newLsm.accelY | (input.read() & 0xFF));
                        newLsm.accelY = (short)(newLsm.accelY | ((input.read() << 8) & 0xFF00));
                        newLsm.accelZ = (short)(newLsm.accelZ | (input.read() & 0xFF));
                        newLsm.accelZ = (short)(newLsm.accelZ | ((input.read() << 8) & 0xFF00));
                        // Get temperature data
                        newLsm.temp = (byte)input.read();
                        newLsms.samples.add(newLsm);
                        break;
                    case 'M':
                        IMUSample newMpu = new IMUSample();
                        // Get sensor ID
                        newMpu.id = (byte)input.read();
                        // Get rate data
                        newMpu.rateX = (short)input.read();
                        newMpu.rateX = (short)((newMpu.rateX << 8) | (input.read() & 0xFF));
                        newMpu.rateY = (short)input.read();
                        newMpu.rateY = (short)((newMpu.rateY << 8) | (input.read() & 0xFF));
                        newMpu.rateZ = (short)input.read();
                        newMpu.rateZ = (short)((newMpu.rateZ << 8) | (input.read() & 0xFF));
                        // Get acceleration data
                        newMpu.accelX = (short)input.read();
                        newMpu.accelX = (short)((newMpu.accelX << 8) | (input.read() & 0xFF));
                        newMpu.accelY = (short)input.read();
                        newMpu.accelY = (short)((newMpu.accelY << 8) | (input.read() & 0xFF));
                        newMpu.accelZ = (short)input.read();
                        newMpu.accelZ = (short)((newMpu.accelZ << 8) | (input.read() & 0xFF));
                        // Get temperature data
                        newMpu.temp = (short)input.read();
                        newMpu.temp = (short)((newMpu.temp << 8) | (input.read() & 0xFF));
                        newMpus.samples.add(newMpu);
                        break;
                    case 'T':
                        long time = input.read();
                        time = ((time << 8) | (input.read() & 0xFF));
                        time = ((time << 8) | (input.read() & 0xFF));
                        time = ((time << 8) | (input.read() & 0xFF));
                        input.read();
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
