/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <px4_tasks.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/read_uart.h>
#include <uORB/topics/offboard_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>


static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int radar_task;             /**< Handle of daemon task / thread */

static int set_count_mean = 5;


__EXPORT int radar_main(int argc, char *argv[]);

int radar_thread_main(int argc, char *argv[]);

static void usage(const char *reason);


static void usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: radar {start|stop|status|numb} [-p <additional params>]\n\n");
}


int radar_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("daemon already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit = false;
        radar_task = px4_task_spawn_cmd("radar",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         radar_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }


    usage("unrecognized command");
    return 1;

}

int radar_thread_main(int argc, char *argv[])
{

    warnx("[daemon] starting\n");

    thread_running = true;

    int count_position = 0;
    int count_loiter = 0;
    int count_3min = 0;
    int count_data = 0;
    int x_arr[10];
    int y_arr[10];
    int dis_x[4];
    int dis_y[4];

    memset(&x_arr, 0, sizeof(x_arr));
    memset(&y_arr, 0, sizeof(y_arr));

    memset(&dis_x, 0, sizeof(dis_x));
    memset(&dis_y, 0, sizeof(dis_y));

    //switch令牌
    int token = 1;
    int count_loss = 5;

    //计算位置均值
    float32 x_local_sum = 0.0;
    float32 y_local_sum = 0.0;
    float32 z_local_sum = 0.0;
    //计算速度方向
    float32 square_vxy = 0.0;
    //一些设定值高度/vxy
    float32 set_hight = -5.0;
    float32 set_vxy = 0.45;
    //找到图像档位的阈值
    bool count_begin = false;
    bool is_data_useful = true;
    bool get_uart_data = false;

    float32 set_N = 0;
    float32 set_E = 0;
    float32 set_min = 7;


    bool updated_vlp;
    bool updated_ru;
    bool get_home_position = false;


    /*订阅*/
    int read_uart_sub = orb_subscribe(ORB_ID(read_uart));
    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    /*定义offboard话题结构体*/
    struct read_uart_s _read_uart = {};
    struct vehicle_local_position_s _vehicle_local_position = {};
    struct vehicle_local_position_s home_position = {};
//    struct vehicle_local_position_s stay_position = {};
    struct offboard_setpoint_s _offboard_sp = {};

    /*公告主题*/
    /*offboard_pub_pub为handle指针*/
    orb_advert_t offboard_setpoint_pub = orb_advertise(ORB_ID(offboard_setpoint), &_offboard_sp);

    _offboard_sp.ignore_alt_hold = true;

    orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);


    while (!thread_should_exit) {

        _offboard_sp.is_local_frame = true;

        orb_check(vehicle_local_position_sub, &updated_vlp);
        orb_check(read_uart_sub, &updated_ru);

        if (updated_vlp) {
            orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &_vehicle_local_position);

            if (!get_home_position) {
                count_position++;
                x_local_sum = x_local_sum + _vehicle_local_position.x;
                y_local_sum = y_local_sum + _vehicle_local_position.y;
                z_local_sum = z_local_sum + _vehicle_local_position.z;
                if (count_position == 10) {
                    get_home_position = true;
                    home_position.x = x_local_sum/(float)10.0;
                    home_position.y = y_local_sum/(float)10.0;
                    home_position.z = z_local_sum/(float)10.0;
                    count_position = 0;
                }
                printf("get home position\n");
            }

        }
        if (updated_ru) {
            orb_copy(ORB_ID(read_uart), read_uart_sub, &_read_uart);
            if (_read_uart.is_valid) {

                if (get_uart_data) {
                    set_E = _read_uart.datax - (float)160;
                    set_N = (float)120 - _read_uart.datay;
                    printf("set_N=%8.4f\t",(double)set_N);
                    printf("set_E=%8.4f\n",(double)set_E);
                    get_uart_data = false;
                }
                is_data_useful = true;
                count_data = 0;
            } else {
                count_data ++;
            }

            if (count_data > set_count_mean) {
                is_data_useful = false;
                printf("date is useless\n");
            }
        }
        printf("read    x=%6d\t",_read_uart.datax);
        printf("y=%6d\n",_read_uart.datay);
        if (_read_uart.is_valid)
            printf("data valid\n");

        if (count_begin) {
            count_3min++;
        }
        if (count_3min == 1800) {
            token = 6;
        }
        switch (token) {
        case 1://take off
            if (get_home_position) {
                _offboard_sp.ignore_alt_hold = true;
                _offboard_sp.ignore_attitude = true;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = true;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = true;
                _offboard_sp.x = home_position.x;
                _offboard_sp.y = home_position.y;
                _offboard_sp.z = home_position.z + set_hight;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.timestamp = hrt_absolute_time();

                if (_vehicle_local_position.z < (home_position.z + set_hight + (float)0.2)) {
                    count_begin = true;
                    token = 2;
                }
            }
            printf("case 1\t");
            break;

        case 2://悬停获取速度
            _offboard_sp.ignore_alt_hold = false;
            _offboard_sp.ignore_position = false;
            _offboard_sp.is_takeoff_sp = false;
            _offboard_sp.x = home_position.x;
            _offboard_sp.y = home_position.y;
            _offboard_sp.z = home_position.z + set_hight;
            _offboard_sp.timestamp = hrt_absolute_time();

            count_loiter++;
            if (count_loiter == 3)
                get_uart_data = true;
            if (count_loiter == 5) {
                count_loiter = 0;
                token = 3;
            }
            printf("case 2\n");
            break;

        case 3://以某个速度飞向目标
            square_vxy = (float)sqrt(set_E * set_E + set_N * set_N);
            _offboard_sp.ignore_alt_hold = false;
            _offboard_sp.ignore_position = true;
            _offboard_sp.ignore_velocity = false;
            _offboard_sp.is_local_frame = false;
            _offboard_sp.is_loiter_sp = false;

            if (square_vxy < set_min * set_min) {
                _offboard_sp.vx = 0;
                _offboard_sp.vy = 0;
            } else {
                _offboard_sp.vx = set_N / square_vxy * set_vxy;
                _offboard_sp.vy = set_E / square_vxy * set_vxy;
            }

            _offboard_sp.z = home_position.z + set_hight;
            if (_vehicle_local_position.z > home_position.z + set_hight + (float)0.2)
                _offboard_sp.vz = (float)-0.10;
            if (_vehicle_local_position.z < home_position.z + set_hight - (float)0.2)
                _offboard_sp.vz = (float)0.05;
            _offboard_sp.timestamp = hrt_absolute_time();

            count_loiter++;
            if (count_loiter == 3) {
                get_uart_data = true;
                printf("vx = %8.4f\t", (double)_offboard_sp.vx);
                printf("vy = %8.4f\n", (double)_offboard_sp.vy);
            }
            if (count_loiter == 10)
                count_loiter = 0;

            if (!is_data_useful && count_loss < 5) {
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.is_loiter_sp = true;
                count_loss--;
                if (count_loss == 0)
                    count_loss = 5;
            }
            printf("case 3\n");
            break;


        case 6:
            _offboard_sp.ignore_alt_hold = false;
            _offboard_sp.ignore_position = false;
            _offboard_sp.ignore_velocity = true;
            _offboard_sp.is_loiter_sp = false;
            _offboard_sp.is_local_frame = true;
            _offboard_sp.x = home_position.x;
            _offboard_sp.y = home_position.y;
            _offboard_sp.z = home_position.z + set_hight;
            _offboard_sp.vx = 0.0;
            _offboard_sp.vy = 0.0;
            _offboard_sp.vy = 0.0;
            _offboard_sp.timestamp = hrt_absolute_time();

            count_loiter++;
            if (count_loiter == 30) {
                count_loiter = 29;
                if ((_vehicle_local_position.vx * _vehicle_local_position.vx +
                     _vehicle_local_position.vy * _vehicle_local_position.vy) < (float)0.02) {
                    token = 7;
                    count_loiter = 0;
                }
            }
            printf("case 6\n");
            break;

        case 7:
            _offboard_sp.ignore_alt_hold = true;
            _offboard_sp.ignore_position = false;
            _offboard_sp.is_local_frame = true;
            _offboard_sp.is_land_sp =true;
            _offboard_sp.x = home_position.x;
            _offboard_sp.y = home_position.y;
            _offboard_sp.z = home_position.z + (float)1.0;
            _offboard_sp.vx = 0.0;
            _offboard_sp.vy = 0.0;
            _offboard_sp.vy = 0.0;
            _offboard_sp.timestamp = hrt_absolute_time();
            printf("case 7\n");
            break;

        default:
            break;
        }

        orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);
        usleep(100000);
    }

    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}
