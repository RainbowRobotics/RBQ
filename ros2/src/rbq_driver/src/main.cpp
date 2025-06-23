// Copyright 2024 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>
#include <QProcess>
#include <QDebug>
#include <iostream>
static inline int command_terminal_ping_check(const std::string &ip)
{
    int ret=0;
    QProcess *proc;
    proc = new QProcess();
    proc->start("ping", QStringList() << "-c 2" << ip.data());
    proc->waitForFinished(-1);
    QByteArray output = proc->readAllStandardOutput();
    if (!output.isEmpty()){
        if (-1 != QString(output).indexOf("ttl", 0, Qt::CaseInsensitive)){
            std::cout << ip.data() << ", PING OK\n";
            ret = 1;
        }
        else{
            std::cout << ip.data() << ", PING Fail\n";
        }
    }
    delete proc;
    return ret;
}

#include "RobotApiHandler.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "VisionPublisher.h"

bool IS_SIM = false;

void spin(const rclcpp::Node::SharedPtr &_node) {
    rclcpp::spin(_node);
}

bool GLOBAL_KILL_SIGNAL = false;

int main(int argc, char * argv[])
{
    std::string host;
    for (int i = 1; i < argc; ++i) {
        QString arg = argv[i];
        if (arg == "-s") {
            IS_SIM = true;
        }
    }
    if (IS_SIM) {
        host = "127.0.0.1";
        std::cout<<"ROS Simulation Mode (127.0.0.1)"<<std::endl;
    } else {
        host = "192.168.0.10";
        std::cout<<"ROS Robot Mode (192.168.0.10)"<<std::endl;
    }

    bool flag = true;
    while(flag) {
        if(command_terminal_ping_check(host)) {
            flag = false;
        } else {
            qDebug() << "command_terminal_ping_check()" << host.data() << " failed !!! ";
            if(!rclcpp::ok()) {
                return -1;
            }
            sleep(1);
        }
    }


    std::shared_ptr<RobotApiHandler> apiHandler = std::make_shared<RobotApiHandler>(host, 200);

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr publisher = std::make_shared<Publisher>(apiHandler, 5ms);
    std::thread thread_publisher(spin, publisher);

    rclcpp::Node::SharedPtr subscriber = std::make_shared<Subscriber>(apiHandler);
    std::thread thread_subscriber(spin, subscriber);

    rclcpp::Node::SharedPtr vision_publisher = std::make_shared<VisionPublisher>(20ms);
    std::thread thread_vision_publisher(spin, vision_publisher);

    while(rclcpp::ok()) {
        rclcpp::sleep_for(1000ms);
    }

    GLOBAL_KILL_SIGNAL = true;

    rclcpp::shutdown();
    thread_publisher.join();
    thread_subscriber.join();
    thread_vision_publisher.join();

    return 0;
}
