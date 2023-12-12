/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger, Tom Pittenger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See https://ardupilot.org/dev for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Plane.h"

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Plane, &plane, func)


/*
  scheduler table - all regular tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table presnet in each of the
  vehicles to determine the order in which tasks are run.  Convenience
  methods SCHED_TASK and SCHED_TASK_CLASS are provided to build
  entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

FAST_TASK entries are run on every loop even if that means the loop
overruns its allotted time
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    //更新姿态航向参考系统（AHRS）                      
    FAST_TASK(ahrs_update),
    //更新飞机的控制模式
    FAST_TASK(update_control_mode),
    //执行飞行稳定化控制
    FAST_TASK(stabilize),
    //快速设置伺服控制,用于调整飞机的飞行姿态（如俯仰、翻滚和偏航）
    FAST_TASK(set_servos),
    //以每秒50次的频率读取遥控器的任务，每次任务的执行时间限制在100微秒以内，优先级为6,接收遥控信号和指令
    SCHED_TASK(read_radio,             50,    100,   6),
    //以每秒50次的频率检查短时失效保护的任务，每次执行时间不超过100微秒，优先级为9,确保在信号短暂丢失时飞机能够安全操作
    SCHED_TASK(check_short_failsafe,   50,    100,   9),
    //以每秒50次的频率更新飞机的速度和高度信息，每次执行时间限制为200微秒，优先级为12,保持飞机的正确飞行高度和速度
    SCHED_TASK(update_speed_height,    50,    200,  12),
    //以每秒100次的频率更新悬停油门设置的任务，每次执行时间限制在90微秒，优先级为24,保持固定翼飞机在悬停时的稳定性
    SCHED_TASK(update_throttle_hover, 100,     90,  24),
    //用于读取遥控通道的模式开关，频率为每秒7次，每次执行时间不超过100微秒，优先级为27,对于飞机根据遥控器输入切换飞行模式非常重要
    SCHED_TASK_CLASS(RC_Channels,     (RC_Channels*)&plane.g2.rc_channels, read_mode_switch,           7,    100, 27),
    //以每秒50次的频率更新GPS数据的任务，每次执行时间限制在300微秒，优先级为30
    SCHED_TASK(update_GPS_50Hz,        50,    300,  30),
    //以每秒10次的频率更新GPS数据，但每次执行时间更长，为400微秒，优先级为33
    SCHED_TASK(update_GPS_10Hz,        10,    400,  33),
    //以每秒10次的频率执行导航任务，每次执行时间限制在150微秒，优先级为36,负责根据预定航线或者自动飞行任务指导飞机飞行
    SCHED_TASK(navigate,               10,    150,  36),
    //以每秒10次的频率更新罗盘（指南针）数据的任务，每次执行时间限制在200微秒，优先级为39
    SCHED_TASK(update_compass,         10,    200,  39),
    //计算空速误差的任务，频率为每秒10次，每次执行时间限制在100微秒，优先级为42
    SCHED_TASK(calc_airspeed_errors,   10,    100,  42),
    //以每秒10次的频率更新高度数据的任务，每次执行时间限制在200微秒，优先级为45
    SCHED_TASK(update_alt,             10,    200,  45),
    //以每秒10次的频率调整高度目标的任务，每次执行时间限制在200微秒，优先级为48
    SCHED_TASK(adjust_altitude_target, 10,    200,  48),
//如果高级失效保护功能启用
#if AP_ADVANCEDFAILSAFE_ENABLED
    //以每秒10次的频率检查高级失效保护系统，每次执行时间限制在100微秒，优先级为51
    SCHED_TASK(afs_fs_check,           10,    100,  51),
#endif
    //以每秒10次的频率检查扩展卡尔曼滤波器（EKF）的状态，每次执行时间限制在75微秒，优先级为54
    SCHED_TASK(ekf_check,              10,     75,  54),
    //更新地面控制站接收数据，频率为每秒300次，每次执行时间限制在500微秒，优先级为57
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500,  57),
    //更新地面控制站发送数据的任务,频率为每秒300次，每次执行时间限制在750微秒，优先级为60
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750,  60),
//如果伺服继电器事件功能启用
#if AP_SERVORELAYEVENTS_ENABLED
    //以每秒50次的频率更新伺服继电器事件，每次任务最多执行150微秒，优先级为63
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events, 50, 150,  63),
#endif
    //以每秒10次读取电池监控数据的任务，最大耗时300微秒，优先级为66
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read,   10, 300,  66),
    //以每秒50次收集气压计数据的任务，最大耗时150微秒，优先级为69
    SCHED_TASK_CLASS(AP_Baro, &plane.barometer, accumulate,  50, 150,  69),
    //以每秒50次的频率读取测距仪的任务，每次执行时间限制为100微秒，优先级为78
    SCHED_TASK(read_rangefinder,       50,    100, 78),
//如果IC发动机（内燃机）控制功能启用
#if AP_ICENGINE_ENABLED
    //以每秒10次更新内燃机控制的任务，最大耗时100微秒，优先级为81
    SCHED_TASK_CLASS(AP_ICEngine,      &plane.g2.ice_control, update,     10, 100,  81),
#endif
//如果光流感应功能启用
#if AP_OPTICALFLOW_ENABLED
    //以每秒50次的频率更新光流数据的任务，每次执行时间限制为50微秒，优先级为87,光流感应器用于检测地面移动，有助于飞行稳定性
    SCHED_TASK_CLASS(AP_OpticalFlow, &plane.optflow, update,    50,    50,  87),
#endif
    //以每秒执行一次的循环任务，最大耗时400微秒，优先级为90
    SCHED_TASK(one_second_loop,         1,    400,  90),
    //以每秒执行三次的循环任务，最大耗时75微秒，优先级为93
    SCHED_TASK(three_hz_loop,           3,     75,  93),
    //以每秒三次检查长时失效保护的任务，最大耗时400微秒，优先级为96
    SCHED_TASK(check_long_failsafe,     3,    400,  96),
//如果转速测量功能启用
#if AP_RPM_ENABLED
    //以每秒10次更新转速传感器的任务，最大耗时100微秒，优先级为99
    SCHED_TASK_CLASS(AP_RPM,           &plane.rpm_sensor,     update,     10, 100,  99),
#endif
//如果空速自动校准功能启用
#if AP_AIRSPEED_AUTOCAL_ENABLE
    //以秒执行一次的空速比率更新任务，最大耗时100微秒，优先级为102
    SCHED_TASK(airspeed_ratio_update,   1,    100,  102),
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
//如果安装支持启用
#if HAL_MOUNT_ENABLED
    //以每秒50次更新摄像机安装点的任务，最大耗时100微秒，优先级为105
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100, 105),
#endif // HAL_MOUNT_ENABLED
//如果相机控制功能启用
#if AP_CAMERA_ENABLED
    //以每秒50次更新相机控制的任务，最大耗时100微秒，优先级为108
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update,      50, 100, 108),
#endif // CAMERA == ENABLED
    //以每5秒执行一次的日志更新任务，最大耗时100微秒，优先级为111
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100, 111),
    //以每10秒保存一次罗盘数据的任务，最大耗时200微秒，优先级为114
    SCHED_TASK(compass_save,          0.1,    200, 114),
    //以以每秒400次的频率写入全速率日志的任务，最大耗时300微秒，优先级为117
    SCHED_TASK(Log_Write_FullRate,        400,    300, 117),
    //以以每秒10次的频率更新日志的任务，最大耗时300微秒，优先级为120
    SCHED_TASK(update_logging10,        10,    300, 120),
    //以以每秒25次的频率更新日志的任务，最大耗时300微秒，优先级为123
    SCHED_TASK(update_logging25,        25,    300, 123),
//如果滑翔功能启用
#if HAL_SOARING_ENABLED
    //以每秒50次更新滑翔数据的任务，最大耗时400微秒，优先级为126
    SCHED_TASK(update_soaring,         50,    400, 126),
#endif
    //以每秒10次检查降落伞系统的任务，最大耗时200微秒，优先级为129
    SCHED_TASK(parachute_check,        10,    200, 129),
//如果地形数据可用
#if AP_TERRAIN_AVAILABLE
    //以每秒10次更新地形数据的任务，最大耗时200微秒，优先级为132
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200, 132),
#endif // AP_TERRAIN_AVAILABLE
    //以每秒5次的频率更新飞行状态的任务，最大耗时100微秒，优先级为135
    SCHED_TASK(update_is_flying_5Hz,    5,    100, 135),
//如果日志记录功能启用
#if LOGGING_ENABLED == ENABLED
    //以每秒50次执行日志记录周期任务的任务，最大耗时400微秒，优先级为138
    SCHED_TASK_CLASS(AP_Logger,         &plane.logger, periodic_tasks, 50, 400, 138),
#endif
    //以每秒50次的频率更新惯性传感器的周期性任务，最大执行时间为50微秒，优先级为141
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins,    periodic,       50,  50, 141),
//如果自动相关监视广播（ADSB）功能启用
#if HAL_ADSB_ENABLED
    //以每秒10次更新ADSB避障数据的任务，最大耗时100微秒，优先级为144
    SCHED_TASK(avoidance_adsb_update,  10,    100, 144),
#endif
    //以每秒10次读取所有辅助遥控通道的任务，最大耗时200微秒，优先级为147
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200, 147),
//如果按键功能启用
#if HAL_BUTTON_ENABLED
    //以每秒5次的频率更新按键状态，每次执行时间限制为100微秒，优先级为150
    SCHED_TASK_CLASS(AP_Button, &plane.button, update, 5, 100, 150),
#endif
//如果统计功能启用
#if STATS_ENABLED == ENABLED
    //以每秒1次的频率更新统计数据，每次执行时间限制为100微秒，优先级为153
    SCHED_TASK_CLASS(AP_Stats, &plane.g2.stats, update, 1, 100, 153),
#endif
//如果夹持器（Gripper）功能启用
#if AP_GRIPPER_ENABLED
    //以秒10次的频率更新夹持器的状态，每次执行时间限制为75微秒，优先级为156
    SCHED_TASK_CLASS(AP_Gripper, &plane.g2.gripper, update, 10, 75, 156),
#endif
//如果起落架控制功能启用
#if AP_LANDINGGEAR_ENABLED
    //以这个任务将以每秒5次的频率更新起落架状态，每次执行时间限制为50微秒，优先级为159
    SCHED_TASK(landing_gear_update, 5, 50, 159),
#endif
};

//提供对飞控任务调度的访问
void Plane::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

//判断是否为垂起无人机，进行失效保护管理
#if HAL_QUADPLANE_ENABLED
constexpr int8_t Plane::_failsafe_priorities[7];
#else
constexpr int8_t Plane::_failsafe_priorities[6];
#endif

// update AHRS system
void Plane::ahrs_update()
{
    /// 更新飞机的解锁状态，检查系统是否满足解锁条件
    arming.update_soft_armed();

    // 更新姿态航向参考系统（AHRS）提供的飞机姿态、航向和加速度信息
    ahrs.update();

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }

    // calculate a scaled roll limit based on current pitch
    //根据设定的滚转角、俯仰角度，计算出按比例的滚转、俯仰限制
    //设置横滚和俯仰的基础限制值
    roll_limit_cd = aparm.roll_limit_cd;
    pitch_limit_min_cd = aparm.pitch_limit_min_cd;

    bool rotate_limits = true;
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        rotate_limits = false;
    }
#endif
    // 如果需要，根据当前的俯仰和横滚角度调整横滚和俯仰限制
    if (rotate_limits) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min_cd *= fabsf(ahrs.cos_roll());
    }

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    // 更新、汇总用于地面转向和自动起飞的陀螺仪数据。
    // 通过DCM.c转换矩阵与陀螺仪矢量的点积给出了大地系的偏航率
    // 更新地面转向控制和自动起飞时使用的总和陀螺仪读数
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

#if HAL_QUADPLANE_ENABLED
    // check if we have had a yaw reset from the EKF
    quadplane.check_yaw_reset();

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update();
#endif

    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
}

/*
  update 50Hz speed/height controller
  以50Hz的频率更新速度/高度控制器
 */
void Plane::update_speed_height(void)
{
    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif
    if (should_run_tecs) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        ///以50Hz频率调用TECS更新，请注意，无论油门是否被抑制，都将调用这个功能，因为需要在起50Hz飞检测中运行
        TECS_controller.update_50hz();
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
#endif
}


/*
  read and update compass
  读取和更新罗盘数据
 */
void Plane::update_compass(void)
{
    compass.read();
}

/*
  do 10Hz logging
 */
void Plane::update_logging10(void)
{
    bool log_faster = (should_log(MASK_LOG_ATTITUDE_FULLRATE) || should_log(MASK_LOG_ATTITUDE_FAST));
    if (should_log(MASK_LOG_ATTITUDE_MED) && !log_faster) {
        Log_Write_Attitude();
        ahrs.Write_AOA_SSA();
    } else if (log_faster) {
        ahrs.Write_AOA_SSA();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/*
  do 25Hz logging
 */
void Plane::update_logging25(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE logs at 400Hz, MASK_LOG_ATTITUDE_FAST at 25Hz, MASK_LOG_ATTIUDE_MED logs at 10Hz
    // highest rate selected wins
    bool log_faster = should_log(MASK_LOG_ATTITUDE_FULLRATE);
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !log_faster) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_NOTCH_FULLRATE)) {
            AP::ins().write_notch_log_messages();
        }
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        Log_Write_Guided();
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        AP::ins().Write_Vibration();
}


/*
  check for AFS failsafe check
  检查AFS的故障安全检测
 */
#if AP_ADVANCEDFAILSAFE_ENABLED
void Plane::afs_fs_check(void)
{
    afs.check(failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

void Plane::one_second_loop()
{
    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(&rcmap, g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

#if HAL_ADSB_ENABLED
    adsb.set_stall_speed_cm(aparm.airspeed_min * 100); // convert m/s to cm/s
    adsb.set_max_speed(aparm.airspeed_max);
#endif

    if (flight_option_enabled(FlightOptions::ENABLE_DEFAULT_AIRSPEED)) {
        // use average of min and max airspeed as default airspeed fusion with high variance
        ahrs.writeDefaultAirSpeed((float)((aparm.airspeed_min + aparm.airspeed_max)/2),
                                  (float)((aparm.airspeed_max - aparm.airspeed_min)/2));
    }

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // update home position if NOT armed and gps position has
    // changed. Update every 5s at most
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            landing.alt_offset = 0;
    }

    // this ensures G_Dt is correct, catching startup issues with constructors
    // calling the scheduler methods
    if (!is_equal(1.0f/scheduler.get_loop_rate_hz(), scheduler.get_loop_period_s()) ||
        !is_equal(G_Dt, scheduler.get_loop_period_s())) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    const float loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.attitude_control->set_notch_sample_rate(loop_rate);
    }
#endif
    rollController.set_notch_sample_rate(loop_rate);
    pitchController.set_notch_sample_rate(loop_rate);
    yawController.set_notch_sample_rate(loop_rate);
}

void Plane::three_hz_loop()
{
#if AP_FENCE_ENABLED
    fence_check();
#endif
}

void Plane::compass_save()
{
    if (AP::compass().available() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
         */
        compass.save_offsets();
    }
}

#if AP_AIRSPEED_AUTOCAL_ENABLE
/*
  once a second update the airspeed calibration ratio
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    update_current_loc();
}

/*
  read update GPS position - 10Hz update
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else if (!hal.util->was_watchdog_reset()) {
                if (!set_home_persistently(gps.location())) {
                    // silently ignore failure...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main control mode dependent update code
 */
void Plane::update_control_mode(void)
{
    if (control_mode != &mode_auto) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    update_fly_forward();

    control_mode->update();
}


void Plane::update_fly_forward(void)
{
    // ensure we are fly-forward when we are flying as a pure fixed
    // wing aircraft. This helps the EKF produce better state
    // estimates as it can make stronger assumptions
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() &&
        quadplane.tailsitter.is_in_fw_flight()) {
        ahrs.set_fly_forward(true);
        return;
    }

    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        ahrs.set_fly_forward(false);
        return;
    }
#endif

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
        return;
    }

    ahrs.set_fly_forward(true);
}

/*
  set the flight stage
 */
void Plane::set_flight_stage(AP_FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    landing.handle_flight_stage_change(fs == AP_FixedWing::FlightStage::LAND);

    if (fs == AP_FixedWing::FlightStage::ABORT_LANDING) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
    Log_Write_Status();
}

//更新飞行器高度
void Plane::update_alt()
{
    //read baro
    yp_read_barometer();
//  barometer.update();

    // calculate the sink rate.
    //计算下沉率
    float sink_rate;
    Vector3f vel;
    // 如果能从AHRS获取速度信息，则使用之
    // 否则，如果GPS的定位质量足够并且有垂直速度信息，则使用GPS的垂直速度
    // 如果以上都不可用，则使用气压计的爬升率（取负值作为下沉率）
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    // 对下沉率进行低通滤波，以减少噪声
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
    // 如果启用了降落伞，则设置降落伞的下沉率
#if PARACHUTE == ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif

    // 更新飞行阶段
    update_flight_stage();

// 如果启用了脚本控制，且脚本控制激活，则不执行TECS（全尺寸能量控制系统）
#if AP_SCRIPTING_ENABLED
    if (nav_scripting_active()) {
        // don't call TECS while we are in a trick
        return;
    }
#endif

    // 判断是否应该运行TECS
    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    // 如果是垂起且应该禁用TECS，则不运行TECS
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif
    
    // 如果应该运行TECS且油门没有被抑制，则进行TECS的更新
    if (should_run_tecs && !throttle_suppressed) {

        // 计算超出着陆航点的距离
        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_FixedWing::FlightStage::LAND &&
            current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        // 设置TECS的目标高度
        tecs_target_alt_cm = relative_target_altitude_cm();

        // 如果是RTL（返回起点）模式且还未完成爬升，则设置一个更高的目标高度以快速爬升
        if (control_mode == &mode_rtl && !rtl.done_climb && (g2.rtl_climb_min > 0 || (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)))) {
            // ensure we do the initial climb in RTL. We add an extra
            // 10m in the demanded height to push TECS to climb
            // quickly
            tecs_target_alt_cm = MAX(tecs_target_alt_cm, prev_WP_loc.alt - home.alt) + (g2.rtl_climb_min+10)*100;
        }

        // 更新TECS的俯仰和油门控制
        TECS_controller.update_pitch_throttle(tecs_target_alt_cm,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor);
    }
}

/*
  recalculate the flight_stage
  飞行阶段
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (control_mode->does_auto_throttle() && !throttle_suppressed) {
        if (control_mode == &mode_auto) {
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
                return;
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Landing aborted via throttle");
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else {
                    set_flight_stage(AP_FixedWing::FlightStage::LAND);
                }
                return;
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        } else if (control_mode != &mode_takeoff) {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        }
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        set_flight_stage(AP_FixedWing::FlightStage::VTOL);
        return;
    }
#endif
    set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
}




/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires
    如果着陆解除延迟被启用（非零），检查是否有着陆，然后在时间结束后自动解锁。
    only called from AP_Landing, when the landing library is ready to disarm
    当着陆库准备解锁时,只从AP_Landing调用。
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm(AP_Arming::Method::AUTOLANDED)) {
                gcs().send_text(MAV_SEVERITY_INFO,"Auto disarmed");
            }
        }
    }
}

bool Plane::trigger_land_abort(const float climb_to_alt_m)
{
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        return quadplane.abort_landing();
    }
#endif

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool is_in_landing = (plane.flight_stage == AP_FixedWing::FlightStage::LAND) ||
        plane.is_land_command(mission_id);
    if (is_in_landing) {
        // fly a user planned abort pattern if available
        if (plane.mission.jump_to_abort_landing_sequence()) {
            return true;
        }

        // only fly a fixed wing abort if we aren't doing quadplane stuff, or potentially
        // shooting a quadplane approach
#if HAL_QUADPLANE_ENABLED
        const bool attempt_go_around =
            (!plane.quadplane.available()) ||
            ((!plane.quadplane.in_vtol_auto()) &&
                (!plane.quadplane.landing_with_fixed_wing_spiral_approach()));
#else
        const bool attempt_go_around = true;
#endif
        if (attempt_go_around) {
            // Initiate an aborted landing. This will trigger a pitch-up and
            // climb-out to a safe altitude holding heading then one of the
            // following actions will occur, check for in this order:
            // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
            //      increment mission index to execute it
            // - else if DO_LAND_START is available, jump to it
            // - else decrement the mission index to repeat the landing approach

            if (!is_zero(climb_to_alt_m)) {
                plane.auto_state.takeoff_altitude_rel_cm = climb_to_alt_m * 100;
            }
            if (plane.landing.request_go_around()) {
                plane.auto_state.next_wp_crosstrack = false;
                return true;
            }
        }
    }
    return false;
}


/*
  the height above field elevation that we pass to TECS
  传递给TECS的现场海拔高度
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;
    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        hgt_afe = height_above_target();
        hgt_afe -= rangefinder_correction();
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

// vehicle specific waypoint info helpers
bool Plane::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        distance = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_distance_to_destination() * 0.01 : 0;
        return true;
    }
#endif
    distance = auto_state.wp_distance;
    return true;
}

bool Plane::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        bearing = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_bearing_to_destination() : 0;
        return true;
    }
#endif
    bearing = nav_controller->target_bearing_cd() * 0.01;
    return true;
}

bool Plane::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        xtrack_error = quadplane.using_wp_nav() ? quadplane.wp_nav->crosstrack_error() : 0;
        return true;
    }
#endif
    xtrack_error = nav_controller->crosstrack_error();
    return true;
}

#if AP_SCRIPTING_ENABLED
// set target location (for use by scripting)
bool Plane::set_target_location(const Location &target_loc)
{
    Location loc{target_loc};

    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    // add home alt if needed
    if (loc.relative_alt) {
        loc.alt += plane.home.alt;
        loc.relative_alt = 0;
    }
    plane.set_guided_WP(loc);
    return true;
}

// set target location (for use by scripting)
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
#endif
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

/*
  update_target_location() works in all auto navigation modes
 */
bool Plane::update_target_location(const Location &old_loc, const Location &new_loc)
{
    if (!old_loc.same_loc_as(next_WP_loc)) {
        return false;
    }
    next_WP_loc = new_loc;
    next_WP_loc.change_alt_frame(old_loc.get_alt_frame());

    return true;
}

// allow for velocity matching in VTOL
bool Plane::set_velocity_match(const Vector2f &velocity)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_land_sequence()) {
        quadplane.poscontrol.velocity_match = velocity;
        quadplane.poscontrol.last_velocity_match_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

// allow for override of land descent rate
bool Plane::set_land_descent_rate(float descent_rate)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent()) {
        quadplane.poscontrol.override_descent_rate = descent_rate;
        quadplane.poscontrol.last_override_descent_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}
#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing.
bool Plane::is_landing() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_land_descent()) {
        return true;
    }
#endif
    return control_mode->is_landing();
}

// returns true if vehicle is taking off.
bool Plane::is_taking_off() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_takeoff()) {
        return true;
    }
#endif
    return control_mode->is_taking_off();
}

// correct AHRS pitch for TRIM_PITCH_CD in non-VTOL modes, and return VTOL view in VTOL
void Plane::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        pitch = quadplane.ahrs_view->pitch;
        roll = quadplane.ahrs_view->roll;
        return;
    }
#endif
    pitch = ahrs.pitch;
    roll = ahrs.roll;
    if (!(flight_option_enabled(FlightOptions::OSD_REMOVE_TRIM_PITCH_CD))) {  // correct for TRIM_PITCH_CD
        pitch -= g.pitch_trim_cd * 0.01 * DEG_TO_RAD;
    }
}

/*
  update current_loc Location
 */
void Plane::update_current_loc(void)
{
    have_position = plane.ahrs.get_location(plane.current_loc);

    // re-calculate relative altitude
    ahrs.get_relative_position_D_home(plane.relative_altitude);
    relative_altitude *= -1.0f;
}

// check if FLIGHT_OPTION is enabled
bool Plane::flight_option_enabled(FlightOptions flight_option) const
{
    return g2.flight_options & flight_option;
}

//main函数
AP_HAL_MAIN_CALLBACKS(&plane);
