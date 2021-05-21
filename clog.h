/******************************************************************************

				  版权所有 (C), 2021-2031, 海康智联

 ******************************************************************************
  文 件 名   : clog.h
  版 本 号   : 初稿
  作    者   : 闫朝东
  生成日期   : 2021年4月23日
  最近修改   :
  功能描述   : 日志打印

  修改历史   :
  1.日    期   : 2021年4月23日
	作    者   : 闫朝东
	修改内容   : 创建文件

******************************************************************************/

#ifndef _CLOG_H_
#define _CLOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CLOG_MAX_FILE_SIZE 100*1024*1024 /*每个文件大小*/
#define CLOG_FLUSH_COUNTS 1000 /*每多少条刷新一次*/

#define CLOG_LEVEL_FAULT 1 /*致命级别日志: 主要添加FAULT: */
#define CLOG_LEVEL_ERROR 2 /*错误级别日志: 主要添加ERROR:*/
#define CLOG_LEVEL_WARNING 3 /*告警打印日志*/
#define CLOG_LEVEL_INFO 4 /*信息日志*/
#define CLOG_LEVEL_DEBUG 5 /*信息日志*/


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
