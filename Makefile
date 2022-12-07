all: collect_data IMU_zero

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h debug_printer.h action_definitions.pb.h packager.h 
CMN_OBJS = I2Cdev.o MPU6050.o action_definitions.pb.o packager.o
CLD_OBJS = collect_data.o
IMU_OBJS = IMU_zero.o

# Set DMP FIFO rate to 100Hz.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

LOCAL_IP = $(shell ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | awk '{print $1}')
LOCAL_PORT = 5000

CXXFLAGS = -DDMP_FIFO_RATE=1 -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`

$(CMN_OBJS) $(CLD_OBJS) $(IMU_OBJS): $(HDRS)

collect_data: $(CMN_OBJS) $(CLD_OBJS)
	$(CXX) -o $@ $^ -l wiringPi -lprotobuf -lm -lpthread

IMU_zero: $(CMN_OBJS) $(IMU_OBJS)
	$(CXX) -o $@ $^ -lm -lprotobuf

clean:
	rm -f $(CMN_OBJS) $(CLD_OBJS) $(IMU_OBJS) collect_data IMU_zero

gui_actish_debug:
	@ echo "http://$(LOCAL_IP):$(LOCAL_PORT)"
	@ gdbgui --host $(LOCAL_IP)  --port $(LOCAL_PORT) --args collect_data 0
