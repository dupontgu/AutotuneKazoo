#ifndef Yin_h
#define Yin_h

#include <stdint.h>

/**
 * Source code adapted from RobSmithDev's project: https://www.youtube.com/watch?v=n1wGzWAywdk
 */

// #define YIN_SAMPLING_RATE 38461
#define YIN_SAMPLING_RATE 4000

#define BUFFER_SIZE 512
#define HALF_BUFFER_SIZE 256

/**
 * @struct  Yin
 * @breif	Object to encapsulate the parameters for the Yin pitch detection algorithm 
 */
typedef struct  {
	uint16_t yinBuffer[HALF_BUFFER_SIZE];		/**< Buffer that stores the results of the intermediate processing steps of the algorithm */
	uint16_t threshold;		/**< Allowed uncertainty in the result as a decimal (i.e 0.15 is 15%) */
} Yin ;

/**
 * Initialise the Yin pitch detection object
 * @param yin        Yin pitch detection object to initialise
 * @param threshold  Allowed uncertainty (e.g 0.05 will return a pitch with ~95% probability)
 */
void Yin_init(Yin *yin, float threshold);

/**
 * Runs the Yin pitch detection algortihm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float Yin_getPitch(Yin *yin, uint8_t* buffer);


#endif
