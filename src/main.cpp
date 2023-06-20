#include "INA219.h"
#include <iomanip> 

int main(int argc, char *argv[])
{
    int file;
    char filename[15] = {"/dev/i2c-1"};

    file = open(filename, O_RDWR);

    unsigned int microsecond = 1000000;
    INA219_IIC ina219(file, INA219_I2C_ADDRESS4);

    while(ina219.scan() != true) {
        printf("INA219 scan failed!\n");
        exit(0);
    }
    
    ina219.reset();
    ina219.setShuntSizeInOhms(0.01);
    ina219.setShuntVoltOffset_mV(-0.03);
    ina219.setCalibration_32V_2A();

    while(true)
    {
        float shuntvoltage = ina219.getShuntVoltage_mV();
        float busvoltage = ina219.getBusVoltage_V();
        float loadvoltage = busvoltage + (shuntvoltage / 1000.0);
            
        std::cout << "BusVoltage [V]: " << std::setprecision(3) << busvoltage
                  << " ShuntVoltage [mV]: " << shuntvoltage
                  << " LoadVoltage [V]: " <<  loadvoltage
                  << " Current [mA]: " << ina219.getCurrent_mA()
                  << " Power [mW]: " << std::setprecision(5) << ina219.getPower_mW()
                  << std::endl;

        usleep(0.2 * microsecond);
    }

    return 0;
}