import rosbag
import numpy as np
import matplotlib.pyplot as plt
import navpy

def extractData(bag_name,topic):
    bag = rosbag.Bag(bag_name)

    var1 = []
    var2 = []
    var3 = []
    var4 = []
    var5 = []
    var6 = []

    time = []
    time2 = []

    # set a flag
    Iter = 0
    # loop over the topic to read evey message
    for topic, msg, t in bag.read_messages(topics=topic):
        sec = t.to_nsec()

        if Iter == 0:
            start_time = sec

        if Iter % 500 == 0:
            print(msg.pose.pose.position)
            print('---------------------------------------------------')
            var4.append(msg.pose.pose.position.x)
            var5.append(msg.pose.pose.position.y)
            var6.append(-msg.pose.pose.position.z)
            time2.append((sec-start_time)*1e-9)

        time.append((sec-start_time)*1e-9)
        var1.append(msg.pose.pose.position.x)
        var2.append(msg.pose.pose.position.y)
        var3.append(-msg.pose.pose.position.z)
        Iter += 1
    plt.figure(1)
    plt.plot(var1,var2, label = bag_name + ' ESTIMATE')
    plt.scatter(var4,var5,label = "printed points", color = 'red')
    plt.figure(2)
    plt.plot(time,var3, label = bag_name + ' ESTIMATE')
    plt.scatter(time2, var6, label = "printed points", color = 'red')

def extractDataGPS(bag_name, topic):
    bag = rosbag.Bag(bag_name)

    var1 = []
    var2 = []
    var3 = []

    time = []

    # set a flag
    Iter = 0
    # loop over the topic to read evey message
    for topic, msg, t in bag.read_messages(topics=topic):
        sec         = t.to_nsec()
        if Iter == 0:
            start_time = sec
            initial_lat = msg.latitude
            initial_lon = msg.longitude
            initial_alt = msg.altitude
        pos = navpy.lla2ned(msg.latitude,msg.longitude,msg.altitude,initial_lat,initial_lon,initial_alt)
        x = pos[0]
        y = pos[1]
        z = pos[2]
        time.append((sec-start_time)*1e-9)
        var1.append(x)
        var2.append(y)
        var3.append(-z)
        Iter += 1
    plt.figure(1)
    plt.plot(var1,var2, label = bag_name + ' RAW')
    plt.figure(2)
    plt.plot(time,var3, label = bag_name + ' RAW')

def extractDataUBLOX(bag_name, topic):
    bag = rosbag.Bag(bag_name)

    var1 = []
    var2 = []
    var3 = []

    time = []

    # set a flag
    Iter = 0
    # loop over the topic to read evey message
    for topic, msg, t in bag.read_messages(topics=topic):
        sec         = t.to_nsec()
        if Iter == 0:
            start_time = sec
            initial_lat = msg.lla[0]
            initial_lon = msg.lla[1]
            initial_alt = msg.lla[2]
        pos = navpy.lla2ned(msg.lla[0],msg.lla[1],msg.lla[2],initial_lat,initial_lon,initial_alt)
        x = pos[0]
        y = pos[1]
        z = pos[2]
        time.append((sec-start_time)*1e-9)
        var1.append(x)
        var2.append(y)
        var3.append(-z)
        Iter += 1
    plt.figure(1)
    plt.plot(var1,var2, label = bag_name + ' RAW')
    plt.figure(2)
    plt.plot(time,var3, label = bag_name + ' RAW')

def extractDataINS(bag_name, topic):
    bag = rosbag.Bag(bag_name)

    var1 = []
    var2 = []
    var3 = []

    time = []

    # set a flag
    Iter = 0
    # loop over the topic to read evey message
    for topic, msg, t in bag.read_messages(topics=topic):
        sec         = t.to_nsec()
        if Iter == 0:
            start_time = sec
            x_start = msg.pose.pose.position.x
            y_start = msg.pose.pose.position.y
            z_start = msg.pose.pose.position.z
        time.append((sec-start_time)*1e-9)
        var1.append(msg.pose.pose.position.x-x_start)
        var2.append(msg.pose.pose.position.y-y_start)
        var3.append(-msg.pose.pose.position.z+z_start)
        Iter += 1
    # plt.figure(1)
    # plt.plot(var1,var2, label = bag_name + ' INS')
    plt.figure(2)
    plt.plot(time,var3, label = bag_name + ' INS')

def extractDataBaro(bag_name, topic):
    bag = rosbag.Bag(bag_name)

    var1 = []
    var2 = []
    var3 = []

    time = []

    # set a flag
    Iter = 0
    # loop over the topic to read evey message
    for topic, msg, t in bag.read_messages(topics=topic):
        sec         = t.to_nsec()
        if Iter == 0:
            start_time = sec
        time.append((sec-start_time)*1e-9)
        # var1.append(msg.pose.pose.position.x-x_start)
        # var2.append(msg.pose.pose.position.y-y_start)
        var3.append(msg.altitude)
        Iter += 1
    # plt.figure(1)
    # plt.plot(var1,var2, label = bag_name + ' INS')
    plt.figure(2)
    plt.plot(time,var3, label = bag_name + ' BARO')


bag_name = 'ins_estimator2_May18.bag'
# bag_name = 'ins_estimator_May18.bag'
# # extractData(bag_name,topic)
# bag_name = 'ublox_estimation_May18.bag'
# # extractData(bag_name,topic)
# bag_name = 'ublox_estimation2_May18.bag'
# # extractData(bag_name,topic)
# bag_name = 'ublox_estimation3_May18.bag'
# extractData(bag_name,topic)

# topic = '/PosVelTime'
# extractDataUBLOX(bag_name,topic)

topic = '/gps'
extractDataGPS(bag_name,topic)

topic = '/odom'
extractData(bag_name,topic)

topic = '/baro'
extractDataBaro(bag_name,topic)

topic = '/ins'
extractDataINS(bag_name,topic)

plt.figure(1)
plt.legend()
plt.xlabel('Pos x, m')
plt.ylabel('Pos y, m')
plt.figure(2)
plt.legend()
plt.xlabel('Time, s')
plt.ylabel('Pos z, m')

plt.show()

