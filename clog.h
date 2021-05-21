/******************************************************************************

				  ��Ȩ���� (C), 2021-2031, ��������

 ******************************************************************************
  �� �� ��   : clog.h
  �� �� ��   : ����
  ��    ��   : �Ƴ���
  ��������   : 2021��4��23��
  ����޸�   :
  ��������   : ��־��ӡ

  �޸���ʷ   :
  1.��    ��   : 2021��4��23��
	��    ��   : �Ƴ���
	�޸�����   : �����ļ�

******************************************************************************/

#ifndef _CLOG_H_
#define _CLOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CLOG_MAX_FILE_SIZE 100*1024*1024 /*ÿ���ļ���С*/
#define CLOG_FLUSH_COUNTS 1000 /*ÿ������ˢ��һ��*/

#define CLOG_LEVEL_FAULT 1 /*����������־: ��Ҫ���FAULT: */
#define CLOG_LEVEL_ERROR 2 /*���󼶱���־: ��Ҫ���ERROR:*/
#define CLOG_LEVEL_WARNING 3 /*�澯��ӡ��־*/
#define CLOG_LEVEL_INFO 4 /*��Ϣ��־*/
#define CLOG_LEVEL_DEBUG 5 /*��Ϣ��־*/


#ifndef SYS_ERR
#define SYS_ERR -1
#endif

#ifndef SYS_OK
#define SYS_OK 0
#endif

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

    int CLOG_evn_init(int log_level, char* logpath = NULL);

    void CLOG_evn_uninit();

    void CLOG_env_setlevel(int ilevel);

    void CLOG_writelog_level(int level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
