#pragma once

#include <chrono>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

namespace rbq_sdk {

namespace Thread {

void print_pthread_error(const char *message, int err) {
    printf("%s: (%d) %s\n", message, err, strerror(err));
}

constexpr size_t kRtHardStackSize = 1024 * 1024; // 1 MiB

void print_current_affinity_mask() {
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    if (sched_getaffinity(0, sizeof(cpu_set_t), &allowed) != 0) {
        print_pthread_error("sched_getaffinity failed", errno);
        return;
    }

    printf("process affinity mask: ");
    bool has_any = false;
    for (int i = 0; i < CPU_SETSIZE; ++i) {
        if (CPU_ISSET(i, &allowed)) {
            printf("%d,", i);
            has_any = true;
        }
    }
    if (!has_any) {
        printf("<empty>");
    }
    printf("\n");
}

void try_set_affinity(pthread_attr_t *attr, int cpu_no) {
    if (cpu_no < 0 || cpu_no >= CPU_SETSIZE) {
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_no, &cpuset);
    const int ret = pthread_attr_setaffinity_np(attr, sizeof(cpu_set_t), &cpuset);
    if (ret) {
        // Keep thread creation alive even when affinity pinning is unavailable.
        printf("pthread cpu allocation skipped for cpu %d: (%d) %s\n", cpu_no, ret, strerror(ret));
        print_current_affinity_mask();
    }
}

int generate_nrt_thread(pthread_t &thread_nrt, void *(*thread_func)(void *), const char *name, int cpu_no, void *arg){
    pthread_attr_t attr;
    int ret;

    ret = pthread_attr_init(&attr);
    if(ret){
        print_pthread_error("init pthread attributes failed", ret);
        return false;
    }

    try_set_affinity(&attr, cpu_no);

    ret = pthread_create(&thread_nrt, &attr, thread_func, arg);
    if(ret){
        print_pthread_error("create pthread failed", ret);
        return false;
    }

    ret = pthread_setname_np(thread_nrt, name);
    if(ret){
        print_pthread_error("thread naming failed1", ret);
        return false;
    }

    return true;
}

int generate_rt_thread(pthread_t &thread_rt, void *(*thread_func)(void *), const char *name, int cpu_no, int priority, void *arg){
    struct sched_param param;
    pthread_attr_t attr;
    int ret;

    ret = pthread_attr_init(&attr);
    if(ret){
        print_pthread_error("init pthread attributes failed", ret);
        return false;
    }

    try_set_affinity(&attr, cpu_no);

    ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if(ret){
        print_pthread_error("pthread setschedpolicy failed", ret);
        return false;
    }
    param.sched_priority = priority;
    ret = pthread_attr_setschedparam(&attr, &param);
    if(ret){
        print_pthread_error("pthread setschedparam failed", ret);
        return false;
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if(ret){
        print_pthread_error("pthread setinheritsched failed", ret);
        return false;
    }

    ret = pthread_create(&thread_rt, &attr, thread_func, arg);
    if(ret){
        print_pthread_error("create pthread failed", ret);
        return false;
    }

    ret = pthread_setname_np(thread_rt, name);
    if(ret){
        print_pthread_error("thread naming failed2", ret);
        return false;
    }
    return true;
}

int generate_rt_thread_hard(pthread_t &thread_rt, void *(*thread_func)(void *), const char *name, int cpu_no, int priority, void *arg){
    struct sched_param param;
    pthread_attr_t attr;
    int ret;

    ret = pthread_attr_init(&attr);
    if(ret){
        print_pthread_error("init pthread attributes failed", ret);
        return false;
    }

    ret = pthread_attr_setstacksize(&attr, kRtHardStackSize);
    if(ret){
        print_pthread_error("pthread setstacksize failed", ret);
        return false;
    }


    try_set_affinity(&attr, cpu_no);

    ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if(ret){
        print_pthread_error("pthread setschedpolicy failed", ret);
        return false;
    }
    param.sched_priority = priority;
    ret = pthread_attr_setschedparam(&attr, &param);
    if(ret){
        print_pthread_error("pthread setschedparam failed", ret);
        return false;
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if(ret){
        print_pthread_error("pthread setinheritsched failed", ret);
        return false;
    }

    ret = pthread_create(&thread_rt, &attr, thread_func, arg);
    if(ret){
        print_pthread_error("create pthread failed", ret);
        return false;
    }

    ret = pthread_setname_np(thread_rt, name);
    if(ret){
        print_pthread_error("thread naming failed3", ret);
        return false;
    }
    return true;
}

static void timespec_add_us(struct timespec *t, long us) {
    t->tv_nsec += us*1000;
    if (t->tv_nsec > 1000000000) {
        t->tv_nsec = t->tv_nsec - 1000000000;
        t->tv_sec += 1;
    }
}

static int timespec_cmp(struct timespec *a, struct timespec *b) {
    if (a->tv_sec > b->tv_sec)
        return 1;
    else if (a->tv_sec < b->tv_sec)
        return -1;
    else {
        if (a->tv_nsec > b->tv_nsec)
            return 1;
        else if (a->tv_nsec == b->tv_nsec)
            return 0;
        else
            return -1;
    }
}

static int timediff_us(struct timespec *before, struct timespec *after) {
    return (after->tv_sec - before->tv_sec)*1e6 + (after->tv_nsec - before->tv_nsec)/1000;
}

} // namespace Thread

} // namespace rbq_sdk
