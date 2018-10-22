#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "thread.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE
#include "stdio.h"
extern NET_ADDR addr_pc; 
void InvensenseTask (void *argument);                                 // thread function
osThreadId_t tid_InvensenseTask;                                      // thread id
int32_t enSendQuat = 0; 
volatile uint16_t cntQuat = 0;
    
    
#define MOTION          (0)
#define NO_MOTION       (1)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)
#define DEFAULT_MPU_HZ  (100)
#define COMPASS_READ_MS (10)
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";



struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
     { 1, 0, 0,
       0, 1, 0,
       0, 0, 1}
};
static struct platform_data_s compass_pdata = {
     { 0, 1, 0,
       1, 0, 0,
       0, 0, -1}
};
#define COMPASS_ENABLED 1


//DISPLAY
extern I2C_HandleTypeDef hi2c1;
#define LCD_ADDR (0x27 << 1)
#define LCD_DELAY_MS 5
#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
                                   uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1,
                                    HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr,
                                  sizeof(data_arr), HAL_MAX_DELAY);
    osDelay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(lcd_addr, 0b00110000);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd_addr, 0b00000010);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, 0b00001100);
    // clear display (optional here)
    LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

void initLCD() {
    LCD_Init(LCD_ADDR);

    // set address to 0x00
    LCD_SendCommand(LCD_ADDR, 0b10000000);
    LCD_SendString(LCD_ADDR, "----MPU9250-----");

    // set address to 0x40
    LCD_SendCommand(LCD_ADDR, 0b11000000);
    LCD_SendString(LCD_ADDR, "  over I2C bus");
}


//END of DISPLAY


//#define USE_CAL_HW_REGISTERS 
static inline void run_self_test(void)
{
    char chAngle[60] = "A%1d G%1d C%1d----9250";
    int8_t accuracy;
    char ch[60];
    int result;
    long gyro[3], accel[3];
    result = mpu_run_6500_self_test(gyro, accel, 0); 
    //sprintf(ch,chAngle,(result & 0x2)>>1,(result & 0x1)>>0,(result & 0x4)>>2);
    //LCD_SendCommand(LCD_ADDR, 0b10000000);
    //LCD_SendString(LCD_ADDR, ch);
    if (result == 0x7) {
	MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif

    uint32_t sizes;
    int32_t ret;
    ret = inv_get_mpl_state_size(&sizes);
    ret = inv_load_mpl_states((uint8_t*)&dataMPL[0],sizes);
    }
    else {
            if (!(result & 0x1))
                MPL_LOGE("Gyro failed.\n");
            if (!(result & 0x2))
                MPL_LOGE("Accel failed.\n");
            if (!(result & 0x4))
                MPL_LOGE("Compass failed.\n");
     }

}
	
void InvensenseTask (void *argument) {	
    CLEAR_BIT(GPIOD->ODR,GPIO_PIN_0);
    inv_error_t statInv;
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    osDelay(50);
    SET_BIT(GPIOD->ODR,GPIO_PIN_0);
    osDelay(50);
    //initLCD();
    inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    int_param_s int_param;
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
    result = mpu_init(&int_param);
    if (result) {
        printf("Could not initialize gyro.\n");
    }
    printf("sensor init\n");
     result = inv_init_mpl();
    if (result) {
        printf("Could not initialize MPL.\n");
    }
    printf("MPL init\n");
    statInv = inv_enable_quaternion();
    //inv_init_9x_fusion();
    statInv = inv_enable_9x_sensor_fusion();
    //inv_start_9x_sensor_fusion();
    statInv = inv_enable_fast_nomot();
    statInv = inv_enable_gyro_tc();
    statInv = inv_enable_vector_compass_cal();
    statInv = inv_enable_magnetic_disturbance();
    statInv = inv_enable_eMPL_outputs();
    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            MPL_LOGE("Not authorized.\n");
        }
    }
    if (result) {
        MPL_LOGE("Could not start the MPL.\n");
    }
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#endif
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
                inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
                (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
                inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
                (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
                inv_orientation_matrix_to_scalar(compass_pdata.orientation),
                (long)compass_fsr<<15);
#endif

    /* Compass reads are handled by scheduler. */
    get_tick(&timestamp);



    unsigned char sensors;
    unsigned long sensor_timestamp;    
    short gyro_raw[3], accel_raw[3];
    short compass_short[3];
    long compass[3],accel_long[3];
    long temperature;
    float quatF[4];

    uint8_t more = 1;

    mpu_get_temperature(&temperature, &sensor_timestamp);
    inv_build_temp(temperature, sensor_timestamp);
    run_self_test();
    netQuaternion* quat_buf;
    int8_t accuracy;
    char ch[60];
    long inv_euler[3];
    volatile float inv_eulerF[3];
    volatile int32_t cntLCD = 0;
    int32_t saveMPL=0;
    int32_t read_stat;
		volatile int32_t nsss = netOK;
		double convData = 1. / 65536;
    while (1) {
        do{
            read_stat = mpu_read_fifo(gyro_raw, accel_raw, &sensor_timestamp, &sensors, &more);
            if(read_stat == 0){
                inv_build_gyro(gyro_raw, sensor_timestamp);
                accel_long[0] = (long)accel_raw[0];
                accel_long[1] = (long)accel_raw[1];
                accel_long[2] = (long)accel_raw[2];
                inv_build_accel(&accel_long[0], 0, sensor_timestamp);
                if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                    compass[0] = (long)compass_short[0];
                    compass[1] = (long)compass_short[1];
                    compass[2] = (long)compass_short[2];
                    inv_build_compass(compass, 0, sensor_timestamp);
                }
                inv_execute_on_data();
                inv_get_quaternion_float(&quatF[0]);
								//debug data
								inv_get_sensor_type_euler(&inv_euler[0], &accuracy,(inv_time_t*)&timestamp);
								inv_eulerF[0] = inv_euler[0] * convData;
								inv_eulerF[1] = inv_euler[1] * convData;
								inv_eulerF[2] = inv_euler[2] * convData;			
								//end of debug data
                if(enSendQuat == 1){                    
                    quat_buf = (netQuaternion*)netUDP_GetBuffer (sizeof(netQuaternion));
                    quat_buf->header.type = tQuatsmpl;
                    quat_buf->header.counter = cntQuat++;
                    //memcpy(&quat_buf->w,&quatF[0],16);
										arm_copy_q7((q7_t*)&quatF[0],(q7_t*)&quat_buf->w,16);
										arm_copy_q7((q7_t*)&inv_eulerF[0],(q7_t*)&quat_buf->fiX,12);
                    /*quat_buf->w = quatF[0];
                    quat_buf->x = quatF[1];
                    quat_buf->y = quatF[2];
                    quat_buf->z = quatF[3];*/
                    if(netUDP_Send (udp_sock, &addr_pc , (uint8_t*)quat_buf, sizeof(netQuaternion)) == netOK){
											nsss++;
								
										}
                }
                /*cntLCD++;
                if(cntLCD >10){


                    cntLCD = 0;   
                    sprintf(ch,chAngle,inv_eulerF[0],inv_eulerF[1],inv_eulerF[2]);
                    LCD_SendCommand(LCD_ADDR, 0b11000000);
                    LCD_SendString(LCD_ADDR, ch);
                }*/
            }
        }while(more);
        osDelay(10);
    }
}
//		//inv_get_sensor_type_quat(quatLL,&accuracy,(inv_time_t*)&timestamp);
//		
////		quatInt[0] = (float)quatLL[0] / 65536.;
////		quatInt[1] = (float)quatLL[1] / 65536.;
////		quatInt[2] = (float)quatLL[2] / 65536.;
////		quatInt[3] = (float)quatLL[3] / 65536.;
//		
//		inv_get_quaternion_float(&quatF[0]);
////		inv_get_sensor_type_euler(&quatF[0],&eulerW[0]);
////				
////		inv_get_sensor_type_quat(inv_data, &accuracy, (inv_time_t*)&timestamp);
////		inv_get_sensor_type_gravity(gravity_data, &accuracy, (inv_time_t*)&timestamp);
////		inv_get_sensor_type_rotation_vector(linear_data, &accuracy, (inv_time_t*)&timestamp);
//		inv_get_sensor_type_euler(inv_euler, &accuracy,(inv_time_t*)&timestamp);
//		euler[0] = (float)inv_euler[0] / 65536;
//		euler[1] = (float)inv_euler[1] / 65536;
//		euler[2] = (float)inv_euler[2] / 65536;

//		osDelay(10); 
//                                     // suspend thread
//  }
//}








