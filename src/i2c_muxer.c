#include "i2c_muxer.h"


/**
 * Create 3 x I2C controllers using PIO and management buffers
 * Is it multiplexing or simply shuffling between multiple I2C controllers?
 * 
 * When BYPASSn is High, 
 * Data (byte?, SDA wire state?) from the host i2c (slave) is relayed (buffered?
 * PIO code ?) to the currently "active"/"selected" master I2C (on to its slave)
 * 
 * Need to think through the return path. 
 * 
 */
int setup_i2c_muxer() {
    // Placeholder for I2C multiplexer setup code
        
    return 0;
}