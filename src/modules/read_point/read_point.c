/*
 * read_point_sensor.c
 * 
 * read sensor through point
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <px4_tasks.h>
#include <px4_config.h>
#include <nuttx/sched.h>

#include <uORB/uORB.h>
#include <uORB/topics/read_point.h>
#include <uORB/topics/radar_pos.h>



__EXPORT int read_point_main(int argc, char *argv[]);


static bool thread_should_exit = false; /*Ddemon exit flag*/
static bool thread_running = false;  /*Daemon status flag*/
static int read_point_task;

/**
 * Main loop
 */
int read_point_thread_main(int argc, char *argv[]);

static int point_init(const char * point_name);
static int set_point_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

int set_point_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios point_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &point_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    point_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    point_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&point_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&point_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &point_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int point_init(const char * point_name)
{
    int serial_fd = open(point_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", point_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: read_point_sensor {start|stop|status} [param]\n\n");
    exit(1);
}

int read_point_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[Read_point]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[Read_point]already running\n");
            return 0;
        }

        thread_should_exit = false;
        read_point_task = px4_task_spawn_cmd("read_point",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         read_point_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[Read_point]running");
            return 0;

        } else {
            warnx("[Read_point]stopped");
            return 1;
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}

int read_point_thread_main(int argc, char *argv[])
{

    if (argc < 2) {
        errx(1, "[Read_point]need a serial port name as argument");
        usage("eg:");
    }

    const char *point_name = argv[1];

    warnx("[Read_point]opening port %s", point_name);
    char data = '0';
    char buffer[4] = "";
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int point_read = point_init(point_name);
    if(false == point_read)return -1;
    if(false == set_point_baudrate(point_read,9600)){
        printf("[Read_point]set_point_baudrate is failed\n");
        return -1;
    }
    printf("[Read_point]point init is successful\n");

    thread_running = true;

    /*初始化数据结构体 */
    struct read_point_s _point_data;
    struct radar_pos_s _radar_pos;
    memset(&_point_data, 0, sizeof(_point_data));
    _point_data.is_valid = false;
    /* 公告主题 */
    orb_advert_t read_point_pub = orb_advertise(ORB_ID(read_point), &_point_data);
    orb_publish(ORB_ID(read_point), read_point_pub, &_point_data);
	
    orb_advert_t radar_pos_pub = orb_advertise(ORB_ID(radar_pos), &_radar_pos);
    orb_publish(ORB_ID(radar_pos), radar_pos_pub, &_radar_pos);

    while(!thread_should_exit) {
        read(point_read,&data,1);
        if (data == 'X'){
            for(int i = 0;i <4;++i){
                read(point_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            strncpy(_point_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
            _point_data.datax[0] = atoi(_point_data.datastr);//将字符串转换成整形数据
	    _point_data.is_valid = true;
            if (_point_data.datax[0] > 999)
                _point_data.is_valid = false;
            //printf("[Read_point]sensor.data=%d\n",_point_data.datax);
	    _radar_pos.local_datax = _point_data.datax[0];
	    printf("[Read_point]point_datax=%d\n",_radar_pos.local_datax);
        }
        if (data == 'Y'){
            for(int i = 0;i <4;++i){
                read(point_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            strncpy(_point_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
            _point_data.datay[0] = atoi(_point_data.datastr);//将字符串转换成整形数据
            _point_data.is_valid = true;
            if (_point_data.datay[0] > 999)
                _point_data.is_valid = false;
            _point_data.timestamp = hrt_absolute_time();
            //printf("[Read_point]sensor.data=%d\n",_point_data.datay);
	    _radar_pos.local_datay = _point_data.datay[0];
	    printf("[Read_point]point_datay=%d\n",_radar_pos.local_datay);
            orb_publish(ORB_ID(read_point), read_point_pub, &_point_data);
        }
	if(data == 'A'){//ascii必须单引号;A-I
		for(int i = 0;i <4;++i){
			read(point_read,&data,1);
			buffer[i] = data;
			data = '0';
		}
		strncpy(_point_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
		_point_data.datax[1] = atoi(_point_data.datastr);//将字符串转换成整形数据
		_radar_pos.point_datax[0] = _point_data.datax[1];
		_point_data.is_valid = true;
	    	if (_point_data.datax[1] > 999)
			_point_data.is_valid = false;
		//printf("[Read_point]sensor.data=%d\n",_point_data.datax);
		printf("[Read_point]point_datax[%d]=%d\n",1,_radar_pos.point_datax[0]);
	        _point_data.timestamp = hrt_absolute_time();
	        orb_publish(ORB_ID(read_point), read_point_pub, &_point_data);
	}
	if(data == 'a'){//a-i
		for(int i = 0;i <4;++i){
			read(point_read,&data,1);
			buffer[i] = data;
			data = '0';
		}
		strncpy(_point_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
		_point_data.datay[1] = atoi(_point_data.datastr);//将字符串转换成整形数据
		_radar_pos.point_datay[0] = _point_data.datay[1];
		_point_data.is_valid = true;
	        if (_point_data.datay[1] > 999)
			_point_data.is_valid = false;
		printf("[Read_point]point_datay[%d]=%d\n",1,_radar_pos.point_datay[0]);
                _point_data.timestamp = hrt_absolute_time();
    		orb_publish(ORB_ID(read_point), read_point_pub, &_point_data);
		
	}
	if(data == 'B'){//ascii必须单引号;A-I
	for(int i = 0;i <4;++i){
		read(point_read,&data,1);
		buffer[i] = data;
		data = '0';
	}
	strncpy(_point_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
	_point_data.datax[2] = atoi(_point_data.datastr);//将字符串转换成整形数据
	_radar_pos.point_datax[1] = _point_data.datax[2];
	_point_data.is_valid = true;
    	if (_point_data.datax[2] > 999)
		_point_data.is_valid = false;
	//printf("[Read_point]sensor.data=%d\n",_point_data.datax);
	printf("[Read_point]point_datax[%d]=%d\n",2,_radar_pos.point_datax[1]);
	_point_data.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(read_point), read_point_pub, &_point_data);
}
if(data == 'b'){//a-i
	for(int i = 0;i <4;++i){
		read(point_read,&data,1);
		buffer[i] = data;
		data = '0';
	}
	strncpy(_point_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
	_point_data.datay[2] = atoi(_point_data.datastr);//将字符串转换成整形数据
	_radar_pos.point_datay[1] = _point_data.datay[2];
	_point_data.is_valid = true;
	if (_point_data.datay[2] > 999)
		_point_data.is_valid = false;
	printf("[Read_point]point_datay[%d]=%d\n",2,_radar_pos.point_datay[1]);
	_point_data.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(read_point), read_point_pub, &_point_data);
	
	}
    }

    warnx("[Read_point]exiting");
    thread_running = false;
    close(point_read);

    fflush(stdout);
    return 0;
}
