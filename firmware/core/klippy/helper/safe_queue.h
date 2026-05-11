#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#pragma once
#include <queue>
#include <pthread.h>
#include <cstdint>

using namespace std;

template <typename T>
class SafeQueue
{
public:
    SafeQueue()
    {
        // 锁的初始化
        pthread_mutex_init(&mutex, 0);
        // 线程条件变量的初始化
        pthread_cond_init(&cond, 0);
    }
    ~SafeQueue()
    {
        // 锁的释放
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&cond);
    }

    void push(T t)
    {
        pthread_mutex_lock(&mutex); // 加锁
        q.push(t);
        // 通知变化 notify
        // 由系统唤醒一个线程
        // pthread_cond_signal(&cond);
        // 通知所有的线程
        // pthread_cond_broadcast(&cond);
        pthread_mutex_unlock(&mutex); // 操作完成后解锁
    }
    void pop(T &t)
    {
        pthread_mutex_lock(&mutex); // 加锁
        // queue为空是一直等待，直到下一次push进新的数据
        if (q.empty())
        {
            // 挂起状态，释放锁
            // pthread_cond_wait(&cond, & mutex);
            pthread_mutex_unlock(&mutex);
            return;
        }
        // 被唤醒以后
        t = q.front();
        q.pop();

        pthread_mutex_unlock(&mutex); // 操作完成后解锁
    }
    uint64_t size()
    {
        pthread_mutex_lock(&mutex); // 加锁
        uint64_t size = q.size();
        pthread_mutex_unlock(&mutex); // 操作完成后解锁
        return size;
    }
    void clear()
    {
        pthread_mutex_lock(&mutex); // 加锁
        while (!q.empty())
        {
            q.pop();
        }
        pthread_mutex_unlock(&mutex); // 操作完成后解锁
    }

private:
    // 如何保证对这个队列的操作是线程安全的？引入互斥锁
    queue<T> q;
    pthread_mutex_t mutex;

    // 创建条件变量
    pthread_cond_t cond;
};

#endif // SAFE_QUEUE_H