//
// Created by LK on 2022/4/8.
//

#ifndef SPM_SLAM_ANDROID_LOG_COUT_H
#define SPM_SLAM_ANDROID_LOG_COUT_H

#include "pthread.h"
#include "android/log.h"
#include <unistd.h>
#include <string>

namespace Log{
    class Log_Cout
    {
    public:

        static void *thread_func(void*)
        {
            ssize_t rdsz;
            char buf[1280];
            while((rdsz = read(pfd[0], buf, sizeof buf - 1)) > 0) {
                if(buf[rdsz - 1] == '\n') --rdsz;
                buf[rdsz] = 0;  /* add null-terminator */
                __android_log_write(ANDROID_LOG_DEBUG, tag, buf);
            }
            return nullptr;
        }

        static int start_logger(const char *app_name)
        {
            tag = app_name;

            /* make stdout line-buffered and stderr unbuffered */
            setvbuf(stdout, nullptr, _IOLBF, 0);
            setvbuf(stderr, nullptr, _IONBF, 0);

            /* create the pipe and redirect stdout and stderr */
            pipe(pfd);
            dup2(pfd[1], 1);
            dup2(pfd[1], 2);

            /* spawn the logging thread */
            if(pthread_create(&thr, nullptr, thread_func, nullptr) != 0)
                return -1;
            return 0;
        }


    private:
        static int pfd[2];
        static pthread_t thr;
        static const char *tag ;
    };

    const char * Log_Cout::tag = "from-std-cout";
    int Log_Cout::pfd[2];
    pthread_t Log_Cout::thr;
}


#endif //SPM_SLAM_ANDROID_LOG_COUT_H
