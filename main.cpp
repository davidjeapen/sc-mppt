#include "mbed.h"
#include "mppt.h"
#include <cstdio>

//Initialized constants
#define SAMPLE_SIZE 3
#define PO_DELAY (5 * SAMPLE_SIZE)
#define TRACKING_DELAY 10
#define MAXV 60
#define MAXI 7
#define MPP_VIN_CYCLES 300


typedef std::chrono::duration<long long, std::ratio<1, 1000>> millisecond;

static Mppt mppt;
static int po = PO_DELAY;
static unsigned long long cycles = 0;
static int tracking = TRACKING_DELAY;


static float sample_vin[3] = {0, 0, 0};
static float sample_iin[3] = {0, 0, 0};

static float p_duty[3] = {0, 0, 0};
static float duty[3] = {0, 0, 0};

static float vref[3] = {45, 45, 45};
//3 voltage inputs
static float vin[3] = {0, 0, 0};
//3 Current inputs
static float iin[3] = {0, 0, 0};

//MMP Voltage input
static float mpp_vin[3] = {0,0,0};
//MMP Voltage Duty
static float mpp_duty[3] = {0,0,0};
//Power in (voltage * current)
static float power[3] = {0,0,0};
//Volatage out
static float vout = 0;
//Max current
static float testMaxI = 1.5;
//Min and max volatages
static float vmin = 20;
static float vmax = 112;

DigitalIn boost_en(PB_7);
AnalogIn muxed_therm(PA_0);

//Gets the volatage and current from each solar array
void readADC() {
  iin[0] = mppt.bc0.getIin();
  vin[0] = mppt.bc0.getVin();
  iin[1] = mppt.bc1.getIin();
  vin[1] = mppt.bc1.getVin();
  iin[2] = mppt.bc2.getIin()*0.9; // Not sure why this x0.9 is needed but it fixes the reading
  vin[2] = mppt.bc2.getVin();

  vout = mppt.getVout();
}

void resetPO() {
  memset(&sample_vin, 0, 3 * sizeof(float));
  memset(&sample_iin, 0, 3 * sizeof(float));
  po = PO_DELAY;
}

void resetPID() {
  mppt.bc0.pid.reset();
  mppt.bc1.pid.reset();
  mppt.bc2.pid.reset();
}

void resetPWM() {
    duty[0] = 0.0;
    duty[1] = 0.0;
    duty[2] = 0.0;
    mppt.bc0.pid.pwm_.write(duty[0]);
    mppt.bc1.pid.pwm_.write(duty[1]);
    mppt.bc2.pid.pwm_.write(duty[2]);
}

millisecond get_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      Kernel::Clock::now().time_since_epoch());
}


// POWER MAPPING
// This method is intermittently used to remap the power curve at a given time by cycling through duty cycles.
// The purpose is to ensure the algorithm is at the correct MPP in case a "local" MPP is found outside of the
// true MPP.
// TO DO: Figure out how this can return three floats mpp_vin0, mpp_vin1, and mpp_vin2
    void power_map() {
        // The maximum power needs to be reset before each remap.
        float mpow[3] = {0,0,0};

        // Room for improvement: The method currently always cycles through a specified duty cycle range.
        // The more the duty cycle varies from normal operation, the less efficient the algorithm is. There
        // may be a way to adjust a smaller range according to where the last MPP was found.
        for(int i = 5; i <= 20; i++) {
        
            mppt.bc0.pid.pwm_.write(i * (float)1 / 50);
            mppt.bc1.pid.pwm_.write(i * (float)1 / 50);
            mppt.bc2.pid.pwm_.write(i * (float)1 / 50);
        
            ThisThread::sleep_for(CYCLE_MS);

            //Gets a new input current and volatage from each array
            vin[0] = mppt.bc0.getVin();
            iin[0] = mppt.bc0.getIin();

            vin[1] = mppt.bc1.getVin();
            iin[1] = mppt.bc1.getIin();

            vin[2] = mppt.bc2.getVin();
            iin[2] = mppt.bc2.getIin();


            //Caculates the power input based on current and voltage
            power[0] = vin[0] * iin[0];
            power[1] = vin[1] * iin[1];
            power[2] = vin[2] * iin[2];

            //Gets current batter voltage
            vout = mppt.getVout();

            //Prints volatage, power, and current values to file (see TxtData folder)
            printf("   DC:%5.2f   V: %5.2fV | %5.2fV | %5.2fV   Vout: %5.2fV P: %5.2fW | %5.2fW | %5.2fW   MPP V: %5.2fV | %5.2fV | %5.2fV   MPP P: %5.2fW | %5.2fW | %5.2fW   BE: %i\n",
            i * (float)1 / 50, vin[0], vin[1], vin[2], vout, power[0], power[1], power[2], mpp_vin[0], mpp_vin[1], mpp_vin[2], mpow[0], mpow[1], mpow[2], boost_en.read());
        
        if (power[0] > mpow[0]) {
            mpp_vin[0] = vin[0];
            mpow[0] = power[0];
            mpp_duty[0] = i * (float)1 / 50;
        }
        if (power[1] > mpow[1]) {
            mpp_vin[1] = vin[1];
            mpow[1] = power[1];
            mpp_duty[1] = i * (float)1 / 50;
        }
        if (power[2] > mpow[2]) {
            mpp_vin[2] = vin[2];
            mpow[2] = power[2];
            mpp_duty[2] = i * (float)1 / 50;
        }
        }

    //Print the duty, max power, and mpp volate in
    printf("/////////////////////////////\n// TARGET:\n//\n//  MPP DC:%5.2f |%5.2f |%5.2f\n//  MPP POWER: %5.2fW | %5.2fW | %5.2fW\n//  MPP Vin: %5.2f | %5.2f | %5.2f\n//\n/////////////////////////////\n",
    mpp_duty[0], mpp_duty[1], mpp_duty[2], mpow[0], mpow[1], mpow[2], mpp_vin[0], mpp_vin[1], mpp_vin[2]);

// Testing if this can be done outside of this method
    // mppt.bc0.pid.pwm_.write(mpp_duty[0]);
    // mppt.bc1.pid.pwm_.write(mpp_duty[1]);
    // mppt.bc2.pid.pwm_.write(mpp_duty[2]);

    ThisThread::sleep_for(CYCLE_MS);
}

//This is the main loop
int main() {
//   while (mppt.notInit() || mppt.maxIout.getValue() == -1) {
//     printf("No Message...\n");
//     ThisThread::sleep_for(2s);
//   }

     millisecond current_time, p_time = get_ms();
     float iout_share = 0;
     float maxIout = 0;
     float mvin = 0;
     float pin = 0;

     DigitalOut OV_FLT_RST(PB_6);
     OV_FLT_RST.write(0);
     // TODO: Implement overvoltage fault reset (OV_FLT_RST) to be accessed through CAN,
     // otherwise the MPPT needs a manual button reset
     DigitalOut therm_sel0(PB_5);
     DigitalOut therm_sel1(PB_4);


    while (true) {
        therm_sel0.write(0);
        therm_sel1.write(0);
        
        float therm_data0 = muxed_therm.read();
        // send CAN message of muxed_therm

        therm_sel0.write(0);
        therm_sel1.write(1);

        float therm_data1 = muxed_therm.read();
        // send CAN message of muxed_therm

        therm_sel0.write(1);
        therm_sel1.write(0);

        float therm_data2 = muxed_therm.read();
        // send CAN message of muxed_therm
        
        char therm0[10];
        char therm1[10];
        char therm2[10];

        sprintf(therm0, "%G", therm_data0);

        mppt.send(0x300, therm0, 8);
        
        //vout = mppt.getVout();
        
        // if(vout>112){
        //     mppt.bc0.pid.pwm_.write(0.0);
        //     mppt.bc1.pid.pwm_.write(0.0);
        //     mppt.bc2.pid.pwm_.write(0.0);
        //     printf("Vout exceeds maximum battery voltage, setting duty cycles to zero and waiting 10 seconds.\n\n");
        //     wait_us(10000000);
        // }
        // else if(vout<20){
        //     mppt.bc0.pid.pwm_.write(0.0);
        //     mppt.bc1.pid.pwm_.write(0.0);
        //     mppt.bc2.pid.pwm_.write(0.0);
        //     printf("Vout is less than a realistic battery voltage, setting duty cycles to zero and waiting 10 seconds.\n\n");
        //     wait_us(10000000);

        // }
        // Intermittently remap the power curve to stay on track
        if(cycles++ % MPP_VIN_CYCLES == 0 && tracking){
            power_map();
            p_time = get_ms();
        }

        thread_sleep_until((p_time + CYCLE_MS).count());
        current_time = get_ms();
        readADC();



        // MPPT MODE (DEFAULT, BATTERY IS NOT FULL)
        if (tracking) {
            if (po < SAMPLE_SIZE) {
                sample_vin[0] += vin[0];
                sample_vin[1] += vin[1];
                sample_vin[2] += vin[2];

                sample_iin[0] += iin[0];
                sample_iin[1] += iin[1];
                sample_iin[2] += iin[2];
            }

            if (!po) {

                // Only adjusting vref if the string is getting a solar array input, otherwise 
                // vref should stay constant for a future input. Mostly for testing since all
                // strings will have solar input in competition.
                if(vin[0] > 5){
                    vref[0] += mppt.bc0.PO(sample_vin[0], sample_iin[0]);
                }
                if(vin[1] > 5){
                    vref[1] += mppt.bc1.PO(sample_vin[1], sample_iin[1]);
                }
                if(vin[2] > 5){
                    vref[2] += mppt.bc2.PO(sample_vin[2], sample_iin[2]);
                }
                    resetPO();
            } 
            else
                po--;

            // If power map was just created, set duty to the found mpp_duty. Now after remapping, both vref 
            // and the duty cycle should point each string to the MPP immediately.
            if(cycles % MPP_VIN_CYCLES == 1){
                printf("\nsetting duty cycles to mpp_duty\n\n");
                duty[0] = mpp_duty[0];
                duty[1] = mpp_duty[1];
                duty[2] = mpp_duty[2];
                mppt.bc0.pid.pwm_.write(duty[0]);
                mppt.bc1.pid.pwm_.write(duty[1]);
                mppt.bc2.pid.pwm_.write(duty[2]);
            }
            else{
                duty[0] = mppt.bc0.pid.duty(vref[0], vin[0], MAXV, (current_time - p_time).count());
                duty[1] = mppt.bc1.pid.duty(vref[1], vin[1], MAXV, (current_time - p_time).count());
                duty[2] = mppt.bc2.pid.duty(vref[2], vin[2], MAXV, (current_time - p_time).count());
            }
            float pin = vin[0]*iin[0] + vin[1]*iin[1] + vin[2]*iin[2];
            float iout = pin/vout;
    
            // if (vout > vmax) {

            //     printf("Max voltage exceeded.\n");
            //     tracking--;
            // } 
            // else if (vout < vmin){
            //     printf("Min voltage is not supplied.\n");
            //     tracking--;
            // }
            // else
            //         tracking = TRACKING_DELAY;
            

        //CHANGE BACK WHEN FINAL

        // In testing, a string's output will likely be greater than 1.25A. If this is exceeded 15 cycles
        // in a row, constant current mode should be triggered.
        //if (iout > mppt.maxIout.getValue()) { 
                // if (iout > testMaxI) {
                //     // printf("MAX CURRENT OF %5.2fA EXCEEDED\n", mppt.maxIout.getValue());
                //     printf("MAX CURRENT EXCEEDED\n");
                //     tracking--;
                //     if (!tracking) {
                //         //maxIout = mppt.maxIout.getValue();
                //         maxIout = testMaxI;

                //         // Marco's code, related to the switch from CC mode back to MPPT mode
                //         // p_duty[0] = duty[0];
                //         // p_duty[1] = duty[1];
                //         // p_duty[2] = duty[2];
                        
                //         resetPID();
                //         resetPO();
                //         printf("/////////////////////////////\n//\n//\n//    ENTERING CONSTANT CURRENT MODE\n//\n//\n/////////////////////////////\n");
                        
                //         //------- POSSIBLE CCM BUG FIX -------//
                //         // this break could prevent the 'p_time = current_time' line and fix the dt from being off on the very next loop?
                //         //break;
                //         // setting duty cycles to zero could reset the algorithm and avoid large fluctuations?
                //         resetPWM();
                //     }
                // } 
                // else
                //         tracking = TRACKING_DELAY;
            }
        else{
            printf("Vout has been over or under Vmax or Vmin. Setting duty cycles to zero and waiting 10 seconds.\n\n");
            resetPWM();
            wait_us(10000000);


            if(vout > vmin && vout < vmax){    
                printf("/////////////////////////////\n//\n//\n//    BACK TO TRACKING\n//\n//\n/////////////////////////////\n");
                tracking = TRACKING_DELAY;
                resetPID();
                resetPO();
            }
        }
        // CONSTANT CURRENT MODE (BATTERY IS RECEIVING TOO MUCH CURRENT)
        // else {
        //     // If maxIout continues to decrease, we want to track that lower value.
        //     // if(mppt.maxIout.getValue() < maxIout){
        //     //     maxIout = mppt.maxIout.getValue();
        //     // }

        //     //CHANGE BACK WHEN FINAL
        //     //iout_share = maxIout/3;
        //     iout_share = testMaxI; // = 1.5A per string for testing purpose

        //     // Making sure this happens only when there is a realistic string input
        //     if(vin[0]>5 && iin[0]>0.05){
        //         duty[0] = mppt.bc0.pid.duty(iout_share, (iin[0] * vin[0]) / vout, MAXI, (current_time - p_time).count());
        //     } else{
        //         duty[0] = 0;
        //         mppt.bc0.pid.pwm_.write(duty[0]);
        //     }
            
        //     if(vin[1]>5 && iin[1]>0.05){
        //         duty[1] = mppt.bc1.pid.duty(iout_share, iin[1] * vin[1] / vout, MAXI, (current_time - p_time).count());
        //     } else{
        //         duty[1] = 0;
        //         mppt.bc1.pid.pwm_.write(duty[1]);
        //     }

        //     if(vin[2]>5 && iin[2]>0.05){
        //         duty[2] = mppt.bc2.pid.duty(iout_share, iin[2] * vin[2] / vout, MAXI, (current_time - p_time).count());
        //     } else{
        //         duty[2] = 0;
        //         mppt.bc2.pid.pwm_.write(duty[2]);
        //     }
    
        //     // The condition to change back to MPPT mode.
        //     // If maxIout becomes greater than x1.25 the current maxIout being tracked, return
        //     // to MPPT mode. 
        //     //   if (p_duty[0] < duty[0] || p_duty[1] < duty[1] || p_duty[2] < duty[2]) {  // Marco's if statement
        //     //if(mppt.maxIout.getValue() > maxIout*1.25){
        //     if(testMaxI > maxIout*1.25){    
        //         printf("/////////////////////////////\n//\n//\n//    BACK TO MPPT MODE\n//\n//\n/////////////////////////////\n");
        //         tracking = TRACKING_DELAY;
        //         resetPID();
        //         resetPO();
        //     }
        // }
        p_time = current_time;

        pin = vin[0]*iin[0] + vin[1]*iin[1] + vin[2]*iin[2];
        float iout = pin/vout; 

        //Comment these out if testing PID variables in boost.cpp
        if(tracking){
            printf("DC:%5.2f /%5.2f /%5.2f   Vin: %5.2fV / %5.2fV / %5.2fV   Vref: %5.2fV / %5.2fV / %5.2fV   Vout: %5.2fV   Iin:%5.2fA /%5.2fA /%5.2fA   Iout:%5.2fA   Pin:%5.2fW   BE:%i\n",duty[0],duty[1],duty[2], vin[0], vin[1], vin[2],vref[0],vref[1],vref[2],vout,iin[0],iin[1],iin[2],iout,pin,boost_en.read());
        }
        // else{
        //     printf("(CCM)  DC:%5.2f /%5.2f /%5.2f   Iin:%5.2fA /%5.2fA /%5.2fA   Iref:%5.2fA   Iout: %5.2fA   Vin: %5.2fV / %5.2fV / %5.2fV   Vout: %5.2fV   Pin: %5.2fW   BE: %i\n",duty[0],duty[1],duty[2],iin[0],iin[1],iin[2],iout_share,iout,vin[0],vin[1],vin[2],vout,pin,boost_en.read());
        // }
    }
}