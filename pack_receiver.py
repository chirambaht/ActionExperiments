import socket
import action_definitions_pb2
import datetime
import time

HOST = "192.168.1.75"  # The server's hostname or IP address
# HOST = "192.168.43.77"  # The server's hostname or IP address
PORT = 9022  # The port used by the server
connection_count = 1
lost_packets = 0
got_packets = 0
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
save_file = open("packets.txt", "a")
# Save file header
save_file.write(
    "time, packet_number, accx, accy, accz, gyrx, gyry, gyrz, qw, qx, qy, qz, temp\n"
)
while (True):
    try:

        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print("Waiting for TCP connection %d" % (connection_count))

        # s.bind((HOST, PORT))
        s.connect((HOST, PORT))

        # now = datetime.now()
        # start_time = time.time()
        # end_time = time.time()
        # current_time = now.strftime("%Y%m%d-%H%M%S")

        print("Connected to %s on port %s" % (str(HOST), str(PORT)))

        while (True):
            data_packet = action_definitions_pb2.ActionDataNetworkPackage()
            data = s.recv(4096)

            try:
                data_packet.ParseFromString(data)
                for dat in data_packet.device_data:
                    save_file.write(
                        f" {data_packet.send_time}, {data_packet.packet_number}, {dat.accelerometer.x}, {dat.accelerometer.y}, {dat.accelerometer.z}, {dat.gyroscope.x}, {dat.gyroscope.y}, {dat.gyroscope.z}, {dat.quaternion.w}, {dat.quaternion.x}, {dat.quaternion.y}, {dat.quaternion.z}, {dat.temperature}\n"
                    )

                got_packets += 1

            except Exception as e:
                print("Error: %s" % (e))
                lost_packets += 1
                continue

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        save_file.close()
        print("Lost packets: %d" % (lost_packets))
        print("Got packets: %d" % (got_packets))

        s.close()
        break