#include "bev_lidar_cali/param_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    ParamProcess processor;
    processor.ReadJsonTest("CAM_FRONT");
    processor.ReadJsonTest("CAM_FRONT_LEFT");
    processor.ReadJsonTest("CAM_FRONT_RIGHT");
    processor.ReadJsonTest("CAM_BACK");
    processor.ReadJsonTest("CAM_BACK_LEFT");
    processor.ReadJsonTest("CAM_BACK_RIGHT");
    return 0;
}