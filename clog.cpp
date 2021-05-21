/******************************************************************************

                  ��Ȩ���� (C), 2021-2031, ��������

 ******************************************************************************
  �� �� ��   : clog.cpp
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

#include <fcntl.h>
#include <time.h>
#include <sys/stat.h>
#include <ctime>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <stdarg.h>
#include <unistd.h>

#include "clog.h"

#define CLOG_LEVEL_STR_FAULT "[FAULT]"
#define CLOG_LEVEL_STR_ERROR "[ERROR]" /*������־ǰ�����ERROR*/
#define CLOG_LEVEL_STR_WARING "[WARNING]"
#define CLOG_LEVEL_STR_EVENT "[EVENT]"
#define CLOG_LEVEL_STR_INFO "[INFO]"
#define CLOG_LEVEL_STR_DEBUG "[DEBUG]"

#define CLOG_LEVEL_STR_LEN 8 /*�ַ����룬ǰ�涼����8������*/

typedef struct LOGCONFIG_ {
	int log_level_;
	char log_path[MAX_PATH];
	int max_size;
	int save_days;
}LOGCONFIG;


LOGCONFIG g_logconfig;
#define PRINTDEBUG 0

/*��־Ŀ¼����*/
#define CLOG_DIRNAME	"log/"

#define CLOG_WRITELOG_BUFLEN 8192
#define CLOG_MAX_SIZE 5*1024*1024
#define CLOG_WRITELOG_OFF	0	/*��־�رպ�*/

pthread_mutex_t g_clogpthreadlock;

FILE*   g_plogfile = NULL;
int g_nFileNums = 0; // ��ǰ��־�ļ�����
int g_nLogItems = 0; // ��ǰ�ļ���־����
int g_tmLastDay = -1; // �ϴ�ˢ��ʱ��
bool g_bNeedRename = false;

static char g_acCurLogFile[MAX_PATH] = { 0 };

char g_acCurLogDir[MAX_PATH] = { 0 };

static int	g_ilogLevel = CLOG_LEVEL_ERROR;
unsigned long   g_ulFileSize = 0;

char g_acProcessPath[MAX_PATH] = { 0 };
char g_acProcessName[MAX_PATH] = { 0 };

int CLOG_atoi(char *pcint)
{
    return atoi(pcint);
}


int CLOG_GetCurrentProcessDirectory(char *processdir, char* processname, int len)
{
	char* path_end;

	if (readlink("/proc/self/exe", processdir, len) <= 0)

		return -1;

	path_end = strrchr(processdir, '/');

	if (path_end == NULL)

		return -1;

	++path_end;

	strcpy(processname, path_end);

	*path_end = '\0';

	return (size_t)(path_end - processdir);
}

bool CLOG_DirIsExist(char *pcDir)
{
	if (pcDir == NULL) {
		printf("CLOG_DirIsExist:invalid input\n");
		return false;
	}
	if (access(pcDir, F_OK) == 0) {
		return true;
	}
	return false;
}


bool CLOG_FileIsCreate(char *pcFilePath)
{
	if (pcFilePath == NULL) {
		printf("CLOG_FileIsCreate: invalid input\n");
		return -1;
	}
	FILE* fp = fopen(pcFilePath, "a+");
	if (fp == NULL) {
		printf("open file %s failed:errno=%d", pcFilePath, errno);
		return false;
	}
	return true;
}

int CLOG_DirCreate(char *sPathName)
{
	char dirname[256] = { 0 };
	strcpy(dirname, sPathName);
	int i, len = strlen(dirname);
	for (i = 1; i < len; i++)
	{
		if (dirname[i] == '/')
		{
			dirname[i] = 0;
			if (access(dirname, F_OK) != 0)
			{
				if (mkdir(dirname, 0755) == -1)
				{
					printf("mkdir error, errno = %d\n", errno);
					return -1;
				}
			}
			dirname[i] = '/';
		}
	}

	return 0;
}

bool CLOG_FileIsExist(char *pcFile)
{
	if (pcFile == NULL) {
		printf("CLOG_FileIsExist:Invalid input\n");
		return false;
	}

	if (access(pcFile, F_OK) == 0) return true;
	return false;
}

int CheckFileSize()
{
	int size = ftell(g_plogfile);
	if (size > CLOG_MAX_SIZE) {
		fclose(g_plogfile);
		char newname[MAX_PATH] = { 0 };
		int i = 1;
		while (1) {
			snprintf(newname, MAX_PATH, "%s_%d", g_acCurLogFile, i);
			if (access(newname, F_OK) != 0) {
				rename(g_acCurLogFile, newname);
				break;
			}
		}

		g_plogfile = fopen(g_acCurLogFile, "a+");
		if (g_plogfile == NULL) {
			printf("create file:%s failed, errno=%d\n", errno);
			return -1;
		}
		return 1;
	}


	return 0;
}

int CLOG_CheckAndCreateLogPath()
{
    CLOG_GetCurrentProcessDirectory(g_acProcessPath, g_acProcessName, MAX_PATH);

    // ԭ��־·�����ڽ���·���µ�Log
    strncat(g_acProcessPath, CLOG_DIRNAME, MAX_PATH);
    strncpy(g_acCurLogDir, g_acProcessPath, strlen(g_acProcessPath));
    if (false == CLOG_DirIsExist(g_acCurLogDir))
    {
        if (-1 == CLOG_DirCreate(g_acCurLogDir))
        {
            return SYS_ERR;
        }
    }
	time_t tmNow = time(NULL);
	struct tm* tm_ = localtime(&tmNow);
    snprintf(g_acCurLogFile, MAX_PATH, "%s%s_%04d%02d%02d.log", 
		g_acProcessPath, g_acProcessName, tm_->tm_year + 1900, tm_->tm_mon + 1, tm_->tm_mday);
	// �ļ��������򴴽�
    if (false == CLOG_FileIsExist(g_acCurLogFile))
    {
        if (false == CLOG_FileIsCreate(g_acCurLogFile))
        {
            return SYS_ERR;
        }
	}
	else { // �ļ����ڣ����ж��ļ���С
		g_plogfile = fopen(g_acCurLogFile, "a+");
		if (g_plogfile == NULL) {
			printf("create file:%s failed, errno=%d\n", errno);
			return -1;
		}
		if (CheckFileSize() < 0) {
			printf("check filesize failed\n");
			return -1;
		}
	}

    return 0;
}


void CLOG_FileLock()
{
    //	WaitForSingleObject(g_pMutexhandle, INFINITE);
    pthread_mutex_lock(&g_clogpthreadlock);
}

void CLOG_FileUnLock()
{
    //	ReleaseMutex(g_pMutexhandle);
    pthread_mutex_unlock(&g_clogpthreadlock);
}

int CLOG_evn_init(int log_Level, char* logpath)
{
	if (logpath == NULL) {
		if (-1 == CLOG_CheckAndCreateLogPath())
		{
			printf("create log path failed:%s\n", logpath);
			return -1;
		}
	}

    unsigned int filesize = 0;
    // ��ȡ�ļ����
    g_plogfile = fopen(g_acCurLogFile, "a+");
    if (g_plogfile == NULL)
    {
		printf("create file:%s failed, errno = %d", g_acCurLogFile, errno);
        return -1;
    }
	filesize = ftell(g_plogfile);
	pthread_mutex_init(&g_clogpthreadlock, NULL);

    g_ilogLevel = log_Level;
    return 0;
}

void  CLOG_env_setlevel(int ilevel)
{
    if (ilevel >= CLOG_LEVEL_FAULT
        && ilevel <= CLOG_LEVEL_DEBUG)
    {
        g_ilogLevel = ilevel;
    }
}

void CLOG_evn_uninit()
{
    pthread_mutex_destroy(&g_clogpthreadlock);

    if (NULL != g_plogfile)
    {
        fclose(g_plogfile);
        g_plogfile = NULL;
    }

    return;
}

// �����ļ�������
// 1. ���� 2. ����
void CLOG_ChangeFile(struct tm* tm_)
{
	// �Ѹ����ļ�
	if (CheckFileSize() > 0) return ;
	// ����
	if (g_tmLastDay != tm_->tm_mday) {
		// �����ļ���
		if (g_plogfile != NULL) {
			fclose(g_plogfile);
		}
		snprintf(g_acCurLogFile, MAX_PATH, "%s%s_%04d%02d%02d.log",
			g_acProcessPath, g_acProcessName, tm_->tm_year + 1900, tm_->tm_mon + 1, tm_->tm_mday);
	}
	g_plogfile = fopen(g_acCurLogFile, "a+");
	if (g_plogfile == NULL) {
		printf("create file:%s failed, errno=%d\n", errno);
		return;
	}
}

void CLOG_writelog_level(int level, const char *fmt, ...)
{
    char buffer[CLOG_WRITELOG_BUFLEN] = { 0 };
    //char *p = NULL;
    va_list argptr; //����������� 
    int i = 0;// , fh = 0, result = 0;
    int nlen = 0;

#if CLOG_WRITELOG_OFF
    return;
#endif 

    if (g_ilogLevel < level)
    {
        return;
    }

    memset(buffer, 0, CLOG_WRITELOG_BUFLEN);

	struct timeval tv;
	struct timezone tz;
	struct tm *tm_;

	gettimeofday(&tv, &tz);
	tm_ = localtime(&tv.tv_sec);

	snprintf(buffer, CLOG_WRITELOG_BUFLEN, "[%04d-%02d-%02d %02d:%02d:%02d %03d] ",
		tm_->tm_year + 1900,
		tm_->tm_mon + 1,
		tm_->tm_mday,
		tm_->tm_hour,
		tm_->tm_min,
		tm_->tm_sec,
		tv.tv_usec / 1000);

    i += (int)strlen(buffer);

    switch (level)
    {
    case CLOG_LEVEL_ERROR:
        i += snprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - CLOG_LEVEL_STR_LEN, "<%8s-->", CLOG_LEVEL_STR_ERROR);
        break;
    case CLOG_LEVEL_WARNING:
        i += snprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - CLOG_LEVEL_STR_LEN, "<%8s-->", CLOG_LEVEL_STR_WARING);
        break;

    case CLOG_LEVEL_FAULT:
        i += snprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - CLOG_LEVEL_STR_LEN, "<%8s-->", CLOG_LEVEL_STR_FAULT);
        break;

    case CLOG_LEVEL_INFO:
        i += snprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - CLOG_LEVEL_STR_LEN, "<%8s-->", CLOG_LEVEL_STR_INFO);
        break;
    case CLOG_LEVEL_DEBUG:
        i += snprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - CLOG_LEVEL_STR_LEN, "<%8s-->", CLOG_LEVEL_STR_DEBUG);
        break;
    default:
        i += snprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - CLOG_LEVEL_STR_LEN, "<        -->");
        break;
    }
#if 1
    va_start(argptr, fmt); //��ʼ�����fmt����

	int len = vsnprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i, fmt, argptr);                                              //��ѡ����
    va_end(argptr);

    strncat(buffer, "\r\n", sizeof(buffer));
#else
    va_start(argptr, fmt); //��ʼ�����fmt����
    vsnprintf(buffer + i, CLOG_WRITELOG_BUFLEN - i - 128, fmt, argptr); //��ѡ����
    va_end(argptr);
    buffer[CLOG_WRITELOG_BUFLEN - 1] = '\0';
#endif

#if PRINTDEBUG
    printf("%s\n", buffer);
#endif

    CLOG_FileLock();

    if (NULL != g_plogfile)
    {
        fwrite(buffer, strlen(buffer), 1, g_plogfile);
#if 1     
		g_nLogItems++;
		if ((g_nLogItems % CLOG_FLUSH_COUNTS) == 0)
		{
			fflush(g_plogfile);
			// �ж��Ƿ���Ҫ���ļ�
			CLOG_ChangeFile(tm_);
			g_nLogItems = 0;
		}

		if (g_tmLastDay != tm_->tm_mday)
		{
			g_bNeedRename = true;
			g_tmLastDay = tm_->tm_mday;
		}
#endif
    }

    CLOG_FileUnLock();

    return;
}