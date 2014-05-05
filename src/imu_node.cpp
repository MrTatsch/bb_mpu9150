#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

// Needed when mixing C and C++ code/libraries
#ifdef __cplusplus
    extern "C" {
#endif
        #include "mpu9150.h"
        #include "linux_glue.h"
        #include "local_defaults.h"

#ifdef __cplusplus
    }
#endif


int set_cal(int mag, char *cal_file);
void read_loop(unsigned int sample_rate);
void register_sig_handler();
void sigint_handler(int sig);

int done;

void usage(char *argv_0)
{
    printf("\nUsage: %s [options]\n", argv_0);
    printf("  -b <i2c-bus>          The I2C bus number where the IMU is. The default is 1 to use /dev/i2c-1.\n");
    printf("  -s <sample-rate>      The IMU sample rate in Hz. Range 2-50, default 10.\n");
    printf("  -y <yaw-mix-factor>   Effect of mag yaw on fused yaw data.\n");
    printf("                           0 = gyro only\n");
    printf("                           1 = mag only\n");
    printf("                           > 1 scaled mag adjustment of gyro data\n");
    printf("                           The default is 4.\n");
    printf("  -a <accelcal file>    Path to accelerometer calibration file. Default is ./accelcal.txt\n");
    printf("  -m <magcal file>      Path to mag calibration file. Default is ./magcal.txt\n");
    printf("  -v                    Verbose messages\n");
    printf("  -h                    Show this help\n");

    printf("\nExample: %s -b3 -s20 -y10\n\n", argv_0);

    exit(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  ros::Publisher imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 1000);
  ros::Rate loop_rate(10);

  /* Init the sensor the values are hardcoded at the local_defaults.h file */
    int opt, len;
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	int verbose = 0;
	char *mag_cal_file = NULL;
	char *accel_cal_file = NULL;
	unsigned long loop_delay;
	mpudata_t mpu;
    int gyroSensitivity = 131; //LSB/°/s
    int accelerometerSensitivity = 16384; //LSB/g

    // receive the parameters and process them
    while ((opt = getopt(argc, argv, "b:s:y:a:m:vh")) != -1) {
        switch (opt) {
        case 'b':
            i2c_bus = strtoul(optarg, NULL, 0);

            if (errno == EINVAL)
                usage(argv[0]);

            if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS)
                usage(argv[0]);

            break;

        case 's':
            sample_rate = strtoul(optarg, NULL, 0);

            if (errno == EINVAL){
                printf("EINVAL\n");
                usage(argv[0]);
            }

            if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE){
                printf("sample rate problem\n");
                usage(argv[0]);
            }

            break;

        case 'y':
            yaw_mix_factor = strtoul(optarg, NULL, 0);

            if (errno == EINVAL)
                usage(argv[0]);

            if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
                usage(argv[0]);

            break;

        case 'a':
            len = 1 + strlen(optarg);

            accel_cal_file = (char *)malloc(len);

            if (!accel_cal_file) {
                perror("malloc");
                exit(1);
            }

            strcpy(accel_cal_file, optarg);
            break;

        case 'm':
            len = 1 + strlen(optarg);

            mag_cal_file = (char *)malloc(len);

            if (!mag_cal_file) {
                perror("malloc");
                exit(1);
            }

            strcpy(mag_cal_file, optarg);
            break;

        case 'v':
            verbose = 1;
            break;

        case 'h':
        default:
            usage(argv[0]);
            break;
        }
    }

    // Initialize the MPU-9150
	register_sig_handler();
	mpu9150_set_debug(verbose);
	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);
	set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);
	if (accel_cal_file)
		free(accel_cal_file);
	if (mag_cal_file)
		free(mag_cal_file);
	memset(&mpu, 0, sizeof(mpudata_t));
	if (sample_rate == 0)
		return -1;

    // ROS loop config
	loop_delay = (1000 / sample_rate) - 2;
	printf("\nEntering MPU read loop (ctrl-c to exit)\n\n");
	linux_delay_ms(loop_delay);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
    
  while (ros::ok())
  {
    sensor_msgs::Imu imu_msg;

	if (mpu9150_read(&mpu) == 0) {
        
        imu_msg.header.frame_id = "drone";
        imu_msg.header.stamp = ros::Time::now();
        
        imu_msg.orientation.x = mpu.fusedQuat[QUAT_X];
        imu_msg.orientation.y = mpu.fusedQuat[QUAT_Y];
        imu_msg.orientation.z = mpu.fusedQuat[QUAT_Z];
        imu_msg.orientation.w = mpu.fusedQuat[QUAT_W];

        imu_msg.linear_acceleration.x = mpu.calibratedAccel[VEC3_X]/accelerometerSensitivity;
        imu_msg.linear_acceleration.y = mpu.calibratedAccel[VEC3_Y]/accelerometerSensitivity;
        imu_msg.linear_acceleration.z = mpu.calibratedAccel[VEC3_Z]/accelerometerSensitivity;

        imu_msg.angular_velocity.x = mpu.rawGyro[VEC3_X]/gyroSensitivity;
        imu_msg.angular_velocity.y = mpu.rawGyro[VEC3_Y]/gyroSensitivity;
        imu_msg.angular_velocity.z = mpu.rawGyro[VEC3_Z]/gyroSensitivity;
        
	}
//	linux_delay_ms(loop_delay);
    imuPublisher.publish(imu_msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

int set_cal(int mag, char *cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	if (cal_file) {
		f = fopen(cal_file, "r");
		
		if (!f) {
			perror("open(<cal-file>)");
			return -1;
		}
	}
	else {
		if (mag) {
			f = fopen("./magcal.txt", "r");
		
			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("./accelcal.txt", "r");
		
			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}		
	}

	memset(buff, 0, sizeof(buff));
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);
	
	if (mag) 
		mpu9150_set_mag_cal(&cal);
	else 
		mpu9150_set_accel_cal(&cal);

	return 0;
}

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}
