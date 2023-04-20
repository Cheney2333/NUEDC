/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history    2021-7-23   �����˴���pid�ĺ����ͽӿ�
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V2.0.0     Jly-23-2021     Qylann          1. ���Ӳ��ֺ���
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

		
		
		
		
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}

/**
  * @brief          cascade pid struct data init
  * @param[out]     c_pid: cascade PID struct data point
  * @param[in]      oPID: 0: kp, 1: ki, 2:kd
  * @param[in]      iPID: 0: kp, 1: ki, 2:kd
  * @param[in]      oPID_max_out: outside pid max out
  * @param[in]      oPID_max_iout: outside pid max iout
  * @param[in]      iPID_max_out: inside pid max out
  * @param[in]      iPID_max_iout: inside pid max iout
  * @retval         none
  */
/**
  * @brief          cascade pid struct data init
  * @param[out]     c_pid: ���PID�ṹ����ָ��
  * @param[in]      oPID: 0: kp, 1: ki, 2:kd
  * @param[in]      iPID: 0: kp, 1: ki, 2:kd
  * @param[in]      oPID_max_out: �⻷pid������
  * @param[in]      oPID_max_iout: �⻷pid���������
  * @param[in]      iPID_max_out: �ڻ�pid������
  * @param[in]      iPID_max_iout: �ڻ�pid���������
  * @retval         none
  */
void cascade_PID_init(cascade_pid_t *c_pid, const fp32 oPID[3], const fp32 iPID[3], fp32 oPID_max_out, fp32 oPID_max_iout, fp32 iPID_max_out, fp32 iPID_max_iout)
{
    PID_init(&c_pid->pid_outside, PID_POSITION, oPID, oPID_max_out, oPID_max_iout);
    PID_init(&c_pid->pid_inside, PID_POSITION, iPID, iPID_max_out, iPID_max_iout);
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          cascade pid calculate 
  * @param[out]     c_pid: cascade PID struct data point
  * @param[in]      o_ref: outside feedback data 
  * @param[in]      i_ref: inside feedback data 
  * @param[in]      s_set: outside set point
  * @retval         cascade pid out
  */
/**
  * @brief          ���pid����
  * @param[out]     c_pid: ���PID�ṹ����ָ��
  * @param[in]      o_ref: �⻷��������
  * @param[in]      i_ref: �ڻ���������
  * @param[in]      s_set: �⻷�趨ֵ
  * @retval         ���pid���
  */
fp32 cascade_PID_calc(cascade_pid_t *c_pid, fp32 o_ref, fp32 i_ref, fp32 o_set)
{
    c_pid->s_set = o_set;
    c_pid->s_fdb = o_ref;
    c_pid->v_fdb = i_ref;
    
    c_pid->v_set = PID_calc(&c_pid->pid_outside, c_pid->s_fdb, c_pid->s_set);
    c_pid->out = PID_calc(&c_pid->pid_inside, c_pid->v_fdb, c_pid->v_set);
    
    return c_pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**
  * @brief          cascade pid out clear
  * @param[out]     c_pid: cascade PID struct data point
  * @retval         none
  */
/**
  * @brief          ���pid������
  * @param[out]     c_pid: ���PID�ṹ����ָ��
  * @retval         none
  */
void cascade_PID_clear(cascade_pid_t *c_pid)
{
    PID_clear(&c_pid->pid_outside);
    PID_clear(&c_pid->pid_inside);
}


