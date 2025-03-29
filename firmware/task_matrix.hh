#ifndef __TASK_MATRIX_HH
#define __TASK_MATRIX_HH

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "i2c.hh"
#include "tca8418.hh"

class TaskMatrix
{
public:
    TaskMatrix();
    ~TaskMatrix();

    typedef struct event
    {
        uint8_t code;
        bool press;
        bool gpio;
        bool err;
    } event_t;

    void init(I2C *i2cbus);
    static void task(void *param);

    QueueHandle_t event_queue;

protected:
    I2C *m_i2c;
    TCA8418 m_matrix;
};


#endif
