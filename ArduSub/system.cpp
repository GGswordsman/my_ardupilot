#include "Sub.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    sub.mainloop_failsafe_check();
}

void Sub::init_ardupilot()
{
    // 初始化主板配置
    BoardConfig.init();

// 如果定义了最大CAN协议驱动数，初始化CAN管理器    
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

// 如果启用了统计数据，初始化统计模块
#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

// 如果启用了抓取器，初始化抓取器
// init cargo gripper
#if AP_GRIPPER_ENABLED
    g2.gripper.init();
#endif

    // initialise notify system
    // 初始化通知系统
    notify.init();

    // initialise battery monitor
    // 初始化电池监控
    battery.init();

    // 初始化气压计
    barometer.init();

// 根据不同的飞控板类型进行特定的气压计设置
#if AP_FEATURE_BOARD_DETECT
    // Detection won't work until after BoardConfig.init()
    // 飞控板检测将在BoardConfig.init()初始化之后才能正常工作
    switch (AP_BoardConfig::get_board_type()) {
        // 如果检测到的是Pixhawk2飞控板
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        // 为Pixhawk2设置BARO_EXT_BUS参数的默认值为0
        AP_Param::set_default_by_name("BARO_EXT_BUS", 0);
        break;
        // 如果检测到的是原始Pixhawk飞控板
        case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        // 为原始Pixhawk设置BARO_EXT_BUS参数值为1
        AP_Param::set_by_name("BARO_EXT_BUS", 1);
        break;
    // 如果是其他类型的飞控板
    default:
        // 设置BARO_EXT_BUS参数的默认值为1
        AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
        break;
    }
// 如果没有启用飞控板检测功能，且飞控板不是基于Linux的
#elif CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    // 设置BARO_EXT_BUS参数的默认值为1
    AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
#endif

// 如果启用了温度传感器，设置默认的I2C总线
#if AP_TEMPERATURE_SENSOR_ENABLED
    // In order to preserve Sub's previous AP_TemperatureSensor Behavior we set the Default I2C Bus Here
    AP_Param::set_default_by_name("TEMP1_BUS", barometer.external_bus());
#endif

    // setup telem slots with serial ports
    // 用串口设置通信槽
    gcs().setup_uarts();

// 如果启用了日志记录，初始化日志
#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise rc channels including setting mode
    // 初始化遥控通道，包括设置模式
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    // 初始化遥控器输入和输出，以及操纵杆
    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs
    init_joystick();            // joystick initialization

// 如果启用了继电器，初始化继电器
#if AP_RELAY_ENABLED
    relay.init();
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     * // 设置“主循环死亡”检查
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    // 初始化GPS
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    // 初始化指南针
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

// 如果启用了空速计，设置日志位
#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

// 如果启用了光流，初始化光流传感器
#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif

// 如果启用了云台控制，初始化云台
#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
    // This step is necessary so that the servo is properly initialized
    camera_mount.set_angle_target(0, 0, 0, false);
    // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
    camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif

// 如果启用了相机控制，初始化相机
#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

// 用户自定义的初始化钩子
#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // Init baro and determine if we have external (depth) pressure sensor
    // 初始化气压计并检测是否有外部深度传感器
    // 设置气压计日志记录位
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    // 校准气压计，参数为false表示不强制重新校准
    barometer.calibrate(false);
    // 更新气压计读数
    barometer.update();

    // 遍历所有气压计实例
    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        // 检查是否有水下类型的气压计（用于深度测量）
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER) {
            // 如果找到水下类型气压计，将其设置为主要气压计
            barometer.set_primary_baro(i);
            // 记录深度传感器的索引
            depth_sensor_idx = i;
            // 标记已检测到深度传感器
            ap.depth_sensor_present = true;
            // 初始化深度传感器的健康状态标志
            sensor_health.depth = barometer.healthy(depth_sensor_idx); // initialize health flag
            // 找到第一个水下类型气压计后停止查找
            break; // Go with the first one we find
        }
    }

// 如果未检测到外部深度传感器，使用默认的气压计设置
    if (!ap.depth_sensor_present) {
        // We only have onboard baro
        // No external underwater depth sensor detected
        // 使用默认的机载气压计
        barometer.set_primary_baro(0);
        // 设置高度测量的噪声值较高，因为读数可能与INS的其余部分不一致
        ahrs.set_alt_measurement_noise(10.0f);  // Readings won't correspond with rest of INS
    } else {
        // 如果检测到外部深度传感器，设置较低的噪声值
        ahrs.set_alt_measurement_noise(0.1f);
    }

    // 初始化漏水检测器
    leak_detector.init();

    // 记录最后一次驾驶员的航向
    last_pilot_heading = ahrs.yaw_sensor;

    // initialise rangefinder
    // 初始化测距仪
#if RANGEFINDER_ENABLED == ENABLED
    init_rangefinder();
#endif

    // initialise AP_RPM library
    // 初始化转速传感器
#if AP_RPM_ENABLED
    rpm_sensor.init();
#endif

    // initialise mission library
    // 初始化任务库
    mission.init();

    // initialise AP_Logger library
    // 初始化日志记录器
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));
#endif

    // 地面校准惯性导航系统
    startup_INS_ground();

// 如果启用了脚本，初始化脚本引擎
#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

    // enable CPU failsafe
    // 启用CPU故障保护
    mainloop_failsafe_enable();

    // 设置原始IMU数据的日志记录
    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // flag that initialisation has completed
    // 标记初始化已完成
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Sub::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::SUBMARINE);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if successfully calibrated
// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Sub::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Sub::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    }

    // once armed we require a good absolute position and EKF must not be in const_pos_mode
    return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors.armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    }
    return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
}

/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

// dummy method to avoid linking AFS
#if AP_ADVANCEDFAILSAFE_ENABLED
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#endif

#if HAL_ADSB_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif
