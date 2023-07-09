#include "bev_lidar_cali/param_process.hpp"

using namespace bevlidar;

int main(int argc, char **argv){
    ParamProcess processor;
    processor.ReadJsonTest("CAM_FRONT");
    return 0;
}