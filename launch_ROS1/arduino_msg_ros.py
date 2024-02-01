import serial
import time
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# Replace 'COMx' with the actual port your Arduino Leonardo is connected to
arduino_port = '/dev/ttyACM1'
baud_rate = 115200

# Initialize the ROS node
rospy.init_node('arduino_data_reader', anonymous=True)

# Create a ROS publisher
imu_publisher = rospy.Publisher('arduino_imu_data', Imu, queue_size=10)

# Open the serial connection
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

# Wait for the Arduino to reset
time.sleep(2)
i = 0
data = {"Orientation": {"x": [], "y": [], "z": [], "w": []}, 
        "LinearAcc": {"x": [], "y": [], "z": []}, 
        "AngVel": {"x": [], "y": [], "z": []}}

try:
    while not rospy.is_shutdown():
        # Read data from Arduino
        serial_data = ser.readline().decode('utf-8').rstrip()

        # print(serial_data)
        # print(i)

        imu_msg = Imu()

        # Populate the IMU message
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "adafruit_imu"

            
        if serial_data[0] == 'G':
            idx = serial_data.index('x')
            # print(idx)
            if serial_data[idx+3] == '-':
                # print('x', serial_data[idx+4])
                if serial_data[idx+5] == '.':
                    x = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    x = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+7] == '.':
                    x = float(serial_data[idx+3:idx+10])
            else:
                if serial_data[idx+4] == '.':
                    x = float(serial_data[idx+3:idx+7])
                elif serial_data[idx+5] == '.':
                    x = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    x = float(serial_data[idx+3:idx+9])

            idx = serial_data.index('y')
            # print(idx)
            if serial_data[idx+3] == '-':
                if serial_data[idx+5] == '.':
                    y = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    y = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+7] == '.':
                    y = float(serial_data[idx+3:idx+10])
            else:
                if serial_data[idx+4] == '.':
                    y = float(serial_data[idx+3:idx+7])
                elif serial_data[idx+5] == '.':
                    y = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    y = float(serial_data[idx+3:idx+9])

            idx = serial_data.index('z')
            # print(idx)
            if serial_data[idx+3] == '-':
                if serial_data[idx+5] == '.':
                    z = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    z = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+7] == '.':
                    z = float(serial_data[idx+3:idx+10])
            else:
                if serial_data[idx+4] == '.':
                    z = float(serial_data[idx+3:idx+7])
                elif serial_data[idx+5] == '.':
                    z = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    z = float(serial_data[idx+3:idx+9])

            # print('x:',x, 'y:',y, 'z:',z)

            data['AngVel']['x'] = x
            data['AngVel']['y'] = y
            data['AngVel']['z'] = z


        elif serial_data[0] == 'L':
            idx = serial_data.index('x')
            if serial_data[idx+3] == '-':
                # print('x', serial_data[idx+4])
                if serial_data[idx+5] == '.':
                    x = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    x = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+7] == '.':
                    x = float(serial_data[idx+3:idx+10])
            else:
                if serial_data[idx+4] == '.':
                    x = float(serial_data[idx+3:idx+7])
                elif serial_data[idx+5] == '.':
                    x = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    x = float(serial_data[idx+3:idx+9])

            idx = serial_data.index('y')
            # print(idx)
            if serial_data[idx+3] == '-':
                if serial_data[idx+5] == '.':
                    y = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    y = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+7] == '.':
                    y = float(serial_data[idx+3:idx+10])
            else:
                if serial_data[idx+4] == '.':
                    y = float(serial_data[idx+3:idx+7])
                elif serial_data[idx+5] == '.':
                    y = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    y = float(serial_data[idx+3:idx+9])

            idx = serial_data.index('z')
            # print(idx)
            if serial_data[idx+3] == '-':
                if serial_data[idx+5] == '.':
                    z = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    z = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+7] == '.':
                    z = float(serial_data[idx+3:idx+10])
            else:
                if serial_data[idx+4] == '.':
                    z = float(serial_data[idx+3:idx+7])
                elif serial_data[idx+5] == '.':
                    z = float(serial_data[idx+3:idx+8])
                elif serial_data[idx+6] == '.':
                    z = float(serial_data[idx+3:idx+9])

            # print('x:',x, 'y:',y, 'z:',z)

            data['LinearAcc']['x'] = x
            data['LinearAcc']['y'] = y
            data['LinearAcc']['z'] = z

        
        elif serial_data[0] == 'q':
            idx = serial_data.index('x')
            if serial_data[idx+3] == '-':
                # print('x', serial_data[idx+4])
                if serial_data[idx+5] == '.':
                    x = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    x = float(serial_data[idx+3:idx+11])
                elif serial_data[idx+7] == '.':
                    x = float(serial_data[idx+3:idx+12])
            else:
                if serial_data[idx+4] == '.':
                    x = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+5] == '.':
                    x = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    x = float(serial_data[idx+3:idx+11])

            idx = serial_data.index('y')
            # print(idx)
            if serial_data[idx+3] == '-':
                if serial_data[idx+5] == '.':
                    y = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    y = float(serial_data[idx+3:idx+11])
                elif serial_data[idx+7] == '.':
                    y = float(serial_data[idx+3:idx+12])
            else:
                if serial_data[idx+4] == '.':
                    y = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+5] == '.':
                    y = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    y = float(serial_data[idx+3:idx+11])

            idx = serial_data.index('z')
            # print(idx)
            if serial_data[idx+3] == '-':
                if serial_data[idx+5] == '.':
                    z = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    z = float(serial_data[idx+3:idx+11])
                elif serial_data[idx+7] == '.':
                    z = float(serial_data[idx+3:idx+12])
            else:
                if serial_data[idx+4] == '.':
                    z = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+5] == '.':
                    z = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    z = float(serial_data[idx+3:idx+11])

            idx = serial_data.index('w')
            if serial_data[idx+3] == '-':
                # print('x', serial_data[idx+4])
                if serial_data[idx+5] == '.':
                    w = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    w = float(serial_data[idx+3:idx+11])
                elif serial_data[idx+7] == '.':
                    w = float(serial_data[idx+3:idx+12])
            else:
                if serial_data[idx+4] == '.':
                    w = float(serial_data[idx+3:idx+9])
                elif serial_data[idx+5] == '.':
                    w = float(serial_data[idx+3:idx+10])
                elif serial_data[idx+6] == '.':
                    w = float(serial_data[idx+3:idx+11])

            # print('x:',x, 'y:',y, 'z:',z, 'w:', w)

            data["Orientation"]["x"] = x
            data["Orientation"]["y"] = y
            data["Orientation"]["z"] = z
            data["Orientation"]["w"] = w

        
        
        if i % 4 == 0:

            imu_msg.orientation.x = data["Orientation"]["x"]
            imu_msg.orientation.y = data["Orientation"]["y"]
            imu_msg.orientation.z = data["Orientation"]["z"]
            imu_msg.orientation.w = data["Orientation"]["w"]

            imu_msg.angular_velocity.x = data['AngVel']['x']
            imu_msg.angular_velocity.y = data['AngVel']['y']
            imu_msg.angular_velocity.z = data['AngVel']['z']

            imu_msg.linear_acceleration.x = data['LinearAcc']['x']
            imu_msg.linear_acceleration.y = data['LinearAcc']['y']
            imu_msg.linear_acceleration.z = data['LinearAcc']['z']
            
            # print(i)
            # Print the received data
            # rospy.loginfo(imu_msg)

            # Publish the data to the ROS topic
            # data_publisher.publish(serial_data)
            imu_publisher.publish(imu_msg)

        i += 1

except KeyboardInterrupt:
    # Close the serial connection when the program is interrupted
    ser.close()
    rospy.loginfo("Serial connection closed.")

except rospy.ROSInterruptException:
    # Handle ROS interruption
    ser.close()
    rospy.loginfo("ROS Interrupted. Serial connection closed.")
