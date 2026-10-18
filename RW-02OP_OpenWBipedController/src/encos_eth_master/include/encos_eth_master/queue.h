//
// Created by bismarck on 11/18/22.
//

#ifndef SOEM_QUEUE_H
#define SOEM_QUEUE_H

#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <atomic>
#include <thread>

#include "transmit.h"

using boost::lockfree::spsc_queue, boost::lockfree::capacity;
typedef std::shared_ptr<EtherCAT_Msg> EtherCAT_Msg_ptr;

extern spsc_queue<EtherCAT_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
extern std::atomic<bool> running;
extern std::thread runThread;

#endif //SOEM_QUEUE_H
