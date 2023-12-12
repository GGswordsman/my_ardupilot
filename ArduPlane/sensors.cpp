#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{

    // notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight
    float height;
#if AP_TERRAIN_AVAILABLE
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // use the best available alt estimate via baro above home
        if (flight_stage == AP_FixedWing::FlightStage::LAND) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude;
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    rangefinder_height_update();
}

// return barometric altitude in centimeters
float current_tsst_depth = 0;
void Plane::yp_read_barometer()
{
    barometer.update();
    // If we are reading a positive altitude, the sensor needs calibration
    // Even a few meters above the water we should have no significant depth reading
    // 如果读取到的高度大于0，这意味着传感器需要校准
    // 通常即使在水面几米上方，我们也不应该有显著的深度读数。
    /*    if(barometer.get_altitude() > 0) {
        barometer.update_calibration();
    }*/

    if (yp.depth_sensor_present) {
        // 检查是否存在深度传感器。
        yp_sensor_health.depth = barometer.healthy(yp_depth_sensor_idx);
    }

    if(yp.depth_sensor_present && yp_sensor_health.depth){
    current_tsst_depth = barometer.get_altitude(); 
    test_depth = current_tsst_depth*M_PI/180.0;
    }
    else if(yp.depth_sensor_present){
        test_depth = 3*M_PI/180.0;
    }
    else if(yp_sensor_health.depth){
        test_depth = 2*M_PI/180.0;
    }
    else{
        // test_depth = 1*M_PI/180.0;
        test_depth = yp.depth_sensor_present;
    }
}
