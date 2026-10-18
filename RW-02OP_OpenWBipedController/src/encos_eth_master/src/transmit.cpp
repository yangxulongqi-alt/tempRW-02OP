#include "transmit.h"
extern "C" {
#include "ethercat.h"
#include "motor_control.h"
}

#include "queue.h"
#include <sys/time.h>
#include <cinttypes>
#include <time.h>  // 添加时间头文件
#include <cstdio>
#include <cstring>
#include "time.h"
#define EC_TIMEOUTM

spsc_queue<EtherCAT_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
std::atomic<bool> running{false};
std::thread runThread;

char IOmap[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint64_t num;
bool isConfig[SLAVE_NUMBER]{false};

#define EC_TIMEOUTMON 500

void EtherCAT_Data_Get();

void EtherCAT_Command_Set();

typedef struct {
    uint16_t motor_id;
    float kp;
    float kd;
} MotorCtrlParams;


static void degraded_handler()
{
    printf("[EtherCAT Error] Logging error...\n");
    time_t current_time = time(NULL);
    char *time_str = ctime(&current_time);
    printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
    printf("[EtherCAT Error] Stopping RT process.\n");
}

static int run_ethercat(const char *ifname)
{
    int i;
    int oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    num = 1;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
            if (ec_slavecount < SLAVE_NUMBER)
            {
                printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx+1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_config_map(&IOmap);
            ec_configdc();

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
            {
                printf("[SLAVE %d]\n", slave_idx);
                printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                printf("\n");
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            //expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            expectedWKC = 2;
            printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("[EtherCAT Init] Operational state reached for all slaves.\n");
                inOP = TRUE;
                return 1;
            }
            else
            {
                printf("[EtherCAT Error] Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("[EtherCAT Error] No slaves found!\n");
        }
    }
    else
    {
        printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", ifname);
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 100

static OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    (void)ptr;
    int slave = 0;
    while (1)
    {
        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX)
        {
            // possibly shut down
            printf("[EtherCAT Error] EtherCAT connection degraded.\n");
            printf("[Simulink-Linux] Shutting down....\n");
            degraded_handler();
            break;
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("[EtherCAT Error] Slave %d lost\n", slave);
                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(50000);
    }
}

void EtherCAT_Init(char *ifname)
{
    int i;
    int rc;
    printf("[EtherCAT] Initializing EtherCAT\n");
    osal_thread_create((void *)&checkThread, 128000, (void *)&ecatcheck, (void *)&ctime);
    for (i = 1; i < 10; i++)
    {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 10.\n", i);
        rc = run_ethercat(ifname);
        if (rc)
            break;
        osal_usleep(1000000);
    }
    if (rc)
        printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
    else
    {
        printf("[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
    }
}

void EtherCAT_Transmit(EtherCAT_Msg *MasterCommand)
{
    for (int i = 0; i < ec_slavecount; i++)
    {
        memcpy((void *)(ec_slave[0].outputs + i * sizeof(EtherCAT_Msg)), (void *)&(MasterCommand[i]), sizeof(EtherCAT_Msg));
    }
    ec_send_processdata();
}

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

//数组大小根据从站数量确定
EtherCAT_Msg Rx_Message[SLAVE_NUMBER];
EtherCAT_Msg Tx_Message[SLAVE_NUMBER];
/**
 * @description:
 * @return {*}
 */
void EtherCAT_Run()
{
    static timespec last_time;
    timespec current_time;
    static double freq = 0.0;
    static int count = 0;
    static double total_interval = 0.0;
    
    // 获取当前时间
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    // 计算时间差（单位：秒）
    double interval = (current_time.tv_sec - last_time.tv_sec) +
                     (current_time.tv_nsec - last_time.tv_nsec) * 1e-9;
    
    // 更新最后时间戳
    last_time = current_time;

    // 累计统计
    if(interval > 0) {
        total_interval += interval;
        count++;
        freq += 1.0/interval;
    }

    // 每秒输出一次平均频率
    if(total_interval >= 1.0 && count > 0) {
        printf("[EtherCAT Status] 当前发送频率: %.2f Hz\n", freq/count);
        freq = 0.0;
        count = 0;
        total_interval = 0.0;
    }

    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        printf("[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    // send
    EtherCAT_Command_Set();
    ec_send_processdata();
    // receive
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    EtherCAT_Data_Get();
    //  check for dropped packet
    if (wkc < expectedWKC)
    {
        // printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
    }
    else
    {
        needlf = TRUE;
    }
    wkc_err_iteration_count++;
}

// void EtherCAT_Data_Get()
// {
//     for (int slave = 0; slave < ec_slavecount; ++slave)
//     {
//         EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
//         if (slave_src)
//             Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
//         uint8_t ack_status = RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
//         if (isConfig[slave]) {
//             isConfig[slave] = false;
//             printf("slave %d msg:\n", slave);
//             Rv_Message_Print(ack_status);
//         }
//     }
// }
void EtherCAT_Data_Get()
{
    // Start from slave index 3 (the fourth slave)
    for (int slave = 3; slave < ec_slavecount; ++slave)
    {
        EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        if (slave_src)
            Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        uint8_t ack_status = RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
        if (isConfig[slave]) {
            isConfig[slave] = false;
            printf("slave %d msg:\n", slave);
            Rv_Message_Print(ack_status);
        }
    }
}

#define frequency 1000
#define POS_SPD 3.14/frequency
float pos_set = 0,delta_pos = POS_SPD;
static int i = 0;

void EtherCAT_Command_Set()
{

    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        EtherCAT_Msg_ptr msg;

        // send_motor_ctrl_cmd(&Tx_Message[slave],1,1,0,0,0,0,0);

        // MotorIDReading(&Tx_Message[slave]);
        // MotorCommModeReading(&Tx_Message[slave], 20);
        // get_motor_parameter(&Tx_Message[slave],1, 14, 2);
        set_motor_position(&Tx_Message[slave],1,20,100,20,50,2);
        // set_motor_position(&Tx_Message[slave],4,18,100,20,50,2);
        // set_motor_position(&Tx_Message[slave],5,3,100,20,50,2);
        // set_motor_speed(&Tx_Message[slave], 1,14, 20, 50, 2);
        // set_motor_speed(&Tx_Message[slave], 4,18, -20, 50, 2);
        // set_motor_speed(&Tx_Message[slave], 5,3, -20, 50, 2);
        // set_motor_acceleration(&Tx_Message[slave],1,14,500,1);
        // set_motor_acceleration(&Tx_Message[slave],4,18,500,1);
        // set_motor_acceleration(&Tx_Message[slave],5,3,500,1);








        isConfig[slave] = true;
        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest)
            *(EtherCAT_Msg *)(ec_slave[slave + 1].outputs) = Tx_Message[slave];
    }
}

void runImpl() {
    while (running)
    {
        EtherCAT_Run();
        usleep(1000);
    }
}

void startRun() {
    running = true;
    runThread = std::thread(runImpl);
}

void Ethercat_Motor_Run(MotorCommand* commands, int num_commands) {
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD) {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX) {
        printf("[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    // wkc_err_count = 0;
    // wkc_err_iteration_count = 0;
    process_command(commands, num_commands);
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    EtherCATmotor_Data_Get();



    if (wkc < expectedWKC) {
        // printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
    } else {
        needlf = true;
    }
    // needlf = true;
    wkc_err_iteration_count++;

    // return motor_data;
}

void process_command(MotorCommand* motor_commands, int num_motor_commands) {
    float kp, kd;
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        for (int i = 0; i < num_motor_commands; ++i) {
            MotorCommand motor_command = motor_commands[i];
            if (motor_command.slave_index == slave) {
                switch (motor_command.control_mode) {
                    case 0: {
                        set_motor_position(&Tx_Message[slave],
                                        motor_command.passage,
                                        motor_command.motor_id,
                                        motor_command.position,
                                        motor_command.speed,
                                        1000,
                                        1);
                        break;
                    }
                    case 1:
                        set_motor_speed(&Tx_Message[slave],
                                        motor_command.passage,
                                        motor_command.motor_id,
                                        motor_command.speed,
                                        1000,
                                        1);
                        break;
                    case 2:
                        set_motor_cur_tor(&Tx_Message[slave],
                                        motor_command.passage,
                                        motor_command.motor_id,
                                        motor_command.torque,
                                        1,
                                        1);
                        break;
                    case 3: {
                        //  if (motor_command.motor_id == 14) {
                        //     printf("[DEBUG] passage=%u, motor_id=%u, kp=%.2f, kd=%.2f, pos=%.2f, speed=%.2f, torque=%.2f\n",
                        //                             motor_command.passage,
                        //                             motor_command.motor_id,
                        //                             motor_command.kp,
                        //                             motor_command.kd,
                        //                             motor_command.position,
                        //                             motor_command.speed,
                        //                             motor_command.torque);
                        // }
                        
                        send_motor_ctrl_cmd(&Tx_Message[slave],
                                                motor_command.passage,
                                                motor_command.motor_id,
                                                motor_command.kp, 
                                                motor_command.kd, 
                                                motor_command.position,
                                                motor_command.speed,
                                                motor_command.torque);
                        break;
                    }
                }
            }
        }
        isConfig[slave] = true;
        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest) {
            *slave_dest = Tx_Message[slave];
        } else {
            printf("Failed to get outputs for slave %d\n", slave + 1);
        }
    }
}

void EtherCATmotor_Data_Get() {
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        if (slave_src) {
            Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
            uint8_t ack_status = RV_can_data_repack(&Rx_Message[slave], comm_ack, slave);
        }
    }
}

