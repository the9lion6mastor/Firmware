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
#include <drivers/drv_hrt.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/distance_avoidance.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/offboard_setpoint.h>

static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int avoidance_task;             /**< Handle of daemon task / thread */
static int gps_sequence = 0;


__EXPORT int avoidance_main(int argc, char *argv[]);

int avoidance_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static void usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: avoidance {start|stop|status|numb} [-p <additional params>]\n\n");
}


int avoidance_main(int argc, char *argv[])
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
        avoidance_task = px4_task_spawn_cmd("avoidance",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         avoidance_thread_main,
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

    if (!strcmp(argv[1], "1")) {
        gps_sequence = 1;
        return 0;
    }

    if (!strcmp(argv[1], "2")) {
        gps_sequence = 2;
        return 0;
    }

    usage("unrecognized command");
    return 1;

}

int avoidance_thread_main(int argc, char *argv[])
{

    warnx("[avoidance] starting\n");

    //用于copy home & target  home:1  target:2
    float32 vx = 0.0;
    float32 vy = 0.0;
    float32 sum_x = 0.0;
    float32 sum_y = 0.0;
    float32 sum_z = 0.0;
    float32 b0 = 0.0;
    float32 k0 = 0.0;
    float32 bnow = 0.0;

    //用于判断三次距离是否都大于/小于set
    float32 set_dis_min = 2.0;
    float32 set_dis_max = 3.0;
    float32 dis[3];
    dis[0] = dis[1] = dis[2] = 0.0;
    bool dis_lessthan_set = false;
    bool dis_morethan_set = false;

    //用于计算斜率及目标点,设定一些值
    float32 set_dis_forward = set_dis_min * 2;
    float32 set_dis_around = 1.0;
    float32 set_hight = -2.5;
    float32 set_vx_forward = 0.5;
    float32 set_yaw = 0.0;
    float32 kab;
    float32 kab_v;
    float32 square = 0.0;
    float32 pos_and_neg = 1.0;

    bool get_home = false;
    bool get_target = false;
    bool get_current_a = false;
    bool compute_b = false;
    bool compute_c = false;
    bool compute_d = false;
    bool is_near_target = false;
    bool is_vxy_zero = false;

    int token = 1;

    int count_ave_position = 0;
    int count_dis_real_change = 0;
    int count_loiter = 0;
    int count_loiter_alternate = 0;
    int count_2s = 0;
    int distance_avoidance_sub = orb_subscribe(ORB_ID(distance_avoidance));
    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    struct distance_avoidance_s read_dis;
    struct vehicle_local_position_s position_now;
    struct vehicle_local_position_s position_home;
    struct vehicle_local_position_s position_target;
    struct vehicle_local_position_s position_a;
    struct vehicle_local_position_s position_b;
    struct vehicle_local_position_s position_c;
    struct vehicle_local_position_s position_d;
    struct offboard_setpoint_s _offboard_sp;

    position_a.x = position_a.y = 0.0;
    position_b.x = position_b.y = 0.0;
    position_c.x = position_c.y = 0.0;
    position_d.x = position_d.y = 0.0;

    position_target.x = position_target.y = position_target.z = 0.0;
    position_home.x = position_home.y = position_home.z = 0.0;

    _offboard_sp.ignore_alt_hold = true;
    _offboard_sp.ignore_attitude = true;
    _offboard_sp.ignore_position = true;
    _offboard_sp.ignore_velocity = true;
    _offboard_sp.is_idle_sp = false;
    _offboard_sp.is_land_sp = false;
    _offboard_sp.is_local_frame = true;
    _offboard_sp.is_loiter_sp = false;
    _offboard_sp.is_takeoff_sp = false;
    _offboard_sp.timestamp = hrt_absolute_time();

    orb_advert_t offboard_setpoint_pub = orb_advertise(ORB_ID(offboard_setpoint), &_offboard_sp);

    orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);


    thread_running = true;

    warnx("Hello daemon!\n");
    while (!thread_should_exit) {

        bool updated_avoidance;
        bool updated_position;

        orb_check(distance_avoidance_sub, &updated_avoidance);
        orb_check(vehicle_local_position_sub, &updated_position);

        if (updated_avoidance) {
            orb_copy(ORB_ID(distance_avoidance), distance_avoidance_sub, &read_dis);
            dis[count_dis_real_change%3] = read_dis.current_distance;

            if (dis[0] > read_dis.min_distance && dis[1] > read_dis.min_distance && dis[2] > read_dis.min_distance) {

                if (dis[0] < set_dis_min && dis[1] < set_dis_min && dis[2] < set_dis_min) {
                    dis_lessthan_set = true;
                } else {
                    dis_lessthan_set = false;
                }

                if (dis[0] > set_dis_max && dis[1] > set_dis_max && dis[2] > set_dis_max) {
                    dis_morethan_set = true;
                } else {
                    dis_morethan_set = false;
                }
            }
            printf("dis=%8.4f\n", (double)read_dis.current_distance);
            count_dis_real_change++;
            if (count_dis_real_change == 3)
                count_dis_real_change = 0;
        }
        if (updated_position) {
            orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &position_now);
            vx = position_now.vx;
            vy = position_now.vy;

            if (gps_sequence == 1 && !get_home) {
                sum_x = sum_x + position_now.x;
                sum_y = sum_y + position_now.y;
                sum_z = sum_z + position_now.z;
                count_ave_position++;
                if (count_ave_position == 10) {
                    position_home.x = sum_x/count_ave_position;
                    position_home.y = sum_y/count_ave_position;
                    position_home.z = sum_z/count_ave_position;
                    get_home = true;
                    count_ave_position = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                printf("[gaemon] sequence == home.\n");
                printf("  home:  x=%6.3f\t", (double)position_home.x);
                printf("y=%6.3f\t", (double)position_home.y);
                printf("z=%6.3f\n", (double)position_home.z);

            }

            if (gps_sequence == 2 && !get_target) {
                sum_x = sum_x + position_now.x;
                sum_y = sum_y + position_now.y;
                sum_z = sum_z + position_now.z;
                count_ave_position++;
                if (count_ave_position == 10) {
                    position_target.x = sum_x/count_ave_position;
                    position_target.y = sum_y/count_ave_position;
                    position_target.z = sum_z/count_ave_position;
                    get_target = true;
                    count_ave_position = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                k0 = (position_target.y - position_home.y)
                        / (position_target.x - position_home.x);
                b0 = position_home.y - k0 * position_home.x;
                printf("[gaemon] sequence == target.\n");
                printf("target:  x=%6.3f\t", (double)position_target.x);
                printf("y=%6.3f\t", (double)position_target.y);
                printf("z=%6.3f\n", (double)position_target.z);
            }

            if (get_current_a) {
                sum_x = sum_x + position_now.x;
                sum_y = sum_y + position_now.y;
                sum_z = sum_z + position_now.z;
                count_ave_position++;
                if (count_ave_position == 3) {
                    position_a.x = sum_x/count_ave_position;
                    position_a.y = sum_y/count_ave_position;
                    position_a.z = sum_z/count_ave_position;
                    get_current_a = false;
                    count_ave_position = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                printf("get point a.\n");
            }

        }

        // main loop
        if (get_home && get_target) {
            kab = (position_target.y - position_home.y) / (position_target.x - position_home.x);
            kab_v = (float)-1.0/kab;

            if (compute_b) {//除
                position_b.x = position_a.x
                        + pos_and_neg * set_dis_around / (float)sqrt(1.0 + (double)(kab_v * kab_v));
                position_b.y = position_a.y
                        + pos_and_neg * set_dis_around / (float)sqrt(1.0 + (double)(kab_v * kab_v)) * kab_v;
                compute_b = false;
            }
            if (compute_c) {
                if (position_target.x > position_home.x) {
                    position_c.x = position_b.x
                            + set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab));
                    position_c.y = position_b.y
                            + set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab)) * kab;
                } else {
                    position_c.x = position_b.x
                            - set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab));
                    position_c.y = position_b.y
                            - set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab)) * kab;
                }
                compute_c = false;
            }
            if (compute_d) {
                if (position_target.x > position_home.x) {
                    position_d.x = position_a.x
                            + set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab));
                    position_d.y = position_a.y
                            + set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab)) * kab;
                } else {
                    position_d.x = position_a.x
                            - set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab));
                    position_d.y = position_a.y
                            - set_dis_forward / (float)sqrt(1.0 + (double)(kab * kab)) * kab;
                }
                compute_d = false;
            }

            set_yaw = atan2((double)(position_target.y - position_home.y),
                            (double)(position_target.x - position_home.x));

            square = (position_target.x - position_now.x) * (position_target.x - position_now.x) +
                    (position_target.y - position_now.y) * (position_target.y - position_now.y);
            if (square < (float)9.0)
                is_near_target = true;

            square = vx * vx + vy * vy;
            if (square < (float)0.01) {
                is_vxy_zero = true;
            } else {
                is_vxy_zero = false;
            }
            if (is_near_target && token != 7)
                token = 6;

            switch (token) {
            case 1://takeup
                _offboard_sp.ignore_alt_hold = true;
                _offboard_sp.ignore_attitude = true;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = true;
                _offboard_sp.x = position_home.x;
                _offboard_sp.y = position_home.y;
                _offboard_sp.z = set_hight + position_home.z;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vz = 0.0;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();

                if (position_now.z < (set_hight + position_home.z + (float)0.5)) {

                    count_loiter++;
                    _offboard_sp.ignore_alt_hold = false;
                    _offboard_sp.is_loiter_sp = false;
                    _offboard_sp.is_takeoff_sp = false;
                    _offboard_sp.ignore_attitude = false;
                    if (count_loiter == 3){
                        count_loiter = 0;
                        token = 2;
                    }
                }
                printf("home take off\n");
                break;

            case 2://to target  v-control
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_attitude = false;
                _offboard_sp.ignore_position = true;
                _offboard_sp.ignore_velocity = false;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = position_target.x;
                _offboard_sp.y = position_target.y;
                _offboard_sp.z = set_hight + position_home.z;
                _offboard_sp.vx = set_vx_forward;
                bnow = position_now.y - k0 * position_now.x;
                if (bnow - b0 > (float)0.1)
                    _offboard_sp.vy = -0.2;
                if (bnow - b0 < (float)-0.1)
                    _offboard_sp.vy = 0.2;
                _offboard_sp.vy = 0.0;
                //mark 左飞2s
                if (count_2s < 20)
                    _offboard_sp.vy = 0.05;
                count_2s++;
                if (position_now.z > position_home.z + set_hight + (float)0.2)
                    _offboard_sp.vz = (float)-0.1;
                if (position_now.z < position_home.z + set_hight - (float)0.2)
                    _offboard_sp.vz = (float)0.1;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();

                if (dis_lessthan_set) {
                    count_loiter++;
                    _offboard_sp.is_loiter_sp = true;
                    _offboard_sp.vx = 0.0;
                    if (count_loiter == 2)
                        get_current_a = true;
                    if (count_loiter == 5){
                        compute_b = true;
                        count_loiter = 0;
                        token = 3;
                    }
                }
                printf("vx=%8.4f\t", (double)position_now.vx);
                printf("vy=%8.4f\n", (double)position_now.vy);
                printf("to target\n");
                break;

            case 3://find path(b)
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_attitude = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = position_b.x;
                _offboard_sp.y = position_b.y;
                _offboard_sp.z = set_hight + position_home.z;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vz = 0.0;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();
                count_loiter_alternate++;
                if (count_loiter_alternate > 5) {
                    if (is_vxy_zero) {
                        count_loiter++;
                        _offboard_sp.is_loiter_sp = true;
                        if (count_loiter == 5) {
                            if (!dis_morethan_set) {
                                if (pos_and_neg < 0)
                                    pos_and_neg = pos_and_neg - (float)0.15;
                                pos_and_neg = pos_and_neg * ((float)-1.0);
                                compute_b = true;
                            } else {
                                pos_and_neg = (float)1.0;
                                compute_c = true;
                                token = 4;
                            }
                            count_loiter = 0;
                            count_loiter_alternate = 0;
                        }
                    }
                }
                printf("find path(b)\n");
                break;

            case 4://to c
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_attitude = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = position_c.x;
                _offboard_sp.y = position_c.y;
                _offboard_sp.z = set_hight + position_home.z;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vz = 0.0;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();
                count_loiter++;
                if (count_loiter > 5) {
                    if (is_vxy_zero) {
                        count_loiter = 0;
                        compute_d = true;
                        token = 5;
                    }
                }
                printf("find path(c)\n");
                break;

            case 5://to d
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_attitude = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = position_d.x;
                _offboard_sp.y = position_d.y;
                _offboard_sp.z = set_hight + position_home.z;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vz = 0.0;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();
                count_loiter++;
                if (count_loiter > 5) {
                    if (is_vxy_zero) {
                        count_loiter = 0;
                        token = 2;
                    }
                }
                printf("find path(d)\n");
                break;

            case 6:
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_attitude = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = position_target.x;
                _offboard_sp.y = position_target.y;
                _offboard_sp.z = set_hight + position_home.z;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vz = 0.0;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();


                if (is_vxy_zero) {
                    token = 7;
                    printf("is_zero");
                }
                printf("to target,position\n");
                break;

            case 7:
                _offboard_sp.ignore_alt_hold = true;
                _offboard_sp.ignore_attitude = true;
                _offboard_sp.ignore_position = false;
                _offboard_sp.ignore_velocity = true;
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = true;
                _offboard_sp.is_local_frame = false;
                _offboard_sp.is_loiter_sp = false;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = position_target.x;
                _offboard_sp.y = position_target.y;
                _offboard_sp.z = position_home.z + (float)0.3;
                _offboard_sp.vx = 0.0;
                _offboard_sp.vy = 0.0;
                _offboard_sp.vz = 0.0;
                _offboard_sp.yaw = set_yaw;
                _offboard_sp.timestamp = hrt_absolute_time();

                printf("land\n");
                break;
            default:
                break;
            }
        }
        orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);
        usleep(100000);
    }

    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}
