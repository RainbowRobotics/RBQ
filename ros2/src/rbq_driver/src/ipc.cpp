#include <QDebug>
#include <QSharedMemory>

static inline uint32_t _getTickRead(QSharedMemory *shm)
{
    uint32_t tickRead_;
    memcpy(&tickRead_, (char*)shm->constData()+sizeof(uint32_t), sizeof(uint32_t));
    return tickRead_;
}

static inline void _incrementTickRead(QSharedMemory *shm)
{
    uint32_t tickRead_ = _getTickRead(shm);
    tickRead_++;
    memcpy((char*)shm->data()+sizeof(uint32_t), &tickRead_, sizeof(uint32_t));
}

static inline uint32_t _getTickWrite(QSharedMemory *shm)
{
    uint32_t tickWrite_;
    memcpy(&tickWrite_, (char*)shm->constData(), sizeof(uint32_t));
    return tickWrite_;
}

static inline void _incrementTickWrite(QSharedMemory *shm)
{
    uint32_t tickWrite_ = _getTickWrite(shm);
    tickWrite_++;
    memcpy((char*)shm->data(), &tickWrite_, sizeof(uint32_t));
}

#include "ipc.h"

Vision_IPC::Vision_IPC(QObject *parent) : QObject(parent)
{
    // Default vision sensors: Depth + Point Cloud + ImageColor + ImageIR
    for(int idx = 0; idx < shm_sensors_total; idx++) {
        // Depth
        {
            shm_depth[idx] = new QSharedMemory(shm_depth_key[idx], this);
            if(!shm_depth[idx]->create(sizeof(Vision_IPC::Depth_t), QSharedMemory::ReadWrite) && shm_depth[idx]->error() == QSharedMemory::AlreadyExists) {
                shm_depth[idx]->attach();
                shm_depth_tickRead[idx]       = getTickRead(shm_depth[idx]);
                shm_depth_tickWrite[idx]      = getTickWrite(shm_depth[idx]);
            }
        }
        // Color
        {
            shm_color[idx] = new QSharedMemory(shm_color_key[idx], this);
            if(!shm_color[idx]->create(sizeof(Vision_IPC::ImageColor_t), QSharedMemory::ReadWrite) && shm_color[idx]->error() == QSharedMemory::AlreadyExists) {
                shm_color[idx]->attach();
                shm_color_tickRead[idx]     = getTickRead(shm_color[idx]);
                shm_color_tickWrite[idx]    = getTickWrite(shm_color[idx]);
            }
        }
        // Infrared
        {
            shm_ir[idx] = new QSharedMemory(shm_ir_key[idx], this);
            if(!shm_ir[idx]->create(sizeof(Vision_IPC::ImageIR_t), QSharedMemory::ReadWrite) && shm_ir[idx]->error() == QSharedMemory::AlreadyExists) {
                shm_ir[idx]->attach();
                shm_ir_tickRead[idx]        = getTickRead(shm_ir[idx]);
                shm_ir_tickWrite[idx]       = getTickWrite(shm_ir[idx]);
            }
        }
    }
}

Vision_IPC::~Vision_IPC()
{
    // Default vision sensors: Depth + Point Cloud + ImageColor + ImageIR
    for(int idx = 0; idx < shm_sensors_total; idx++) {
        if(nullptr != shm_depth[idx]) {
            shm_depth[idx]->detach();
            delete shm_depth[idx];
            shm_depth[idx] = nullptr;
            // qDebug() << QString::asprintf("detach, key:%s", shm_depth_key[idx].toLocal8Bit().data());
        }
        if(nullptr != shm_color[idx]) {
            shm_color[idx]->detach();
            delete shm_color[idx];
            shm_color[idx] = nullptr;
            // qDebug() << QString::asprintf("detach, key:%s", shm_color_key[idx].toLocal8Bit().data());
        }
        if(nullptr != shm_ir[idx]) {
            shm_ir[idx]->detach();
            delete shm_ir[idx];
            shm_ir[idx] = nullptr;
            // qDebug() << QString::asprintf("detach, key:%s", shm_ir_key[idx].toLocal8Bit().data());
        }
    }
}

uint32_t Vision_IPC::getTickWrite(QSharedMemory *shm)
{
    uint32_t res = 0;
    if(shm->lock())
    {
        memcpy(&res, (char*)shm->constData(), sizeof(uint32_t));
        shm->unlock();
    }
    return res;
}

uint32_t Vision_IPC::getTickRead(QSharedMemory *shm)
{
    uint32_t res = 0;
    if(shm->lock())
    {
        memcpy(&res, (char*)shm->constData()+sizeof(uint32_t), sizeof(uint32_t));
        shm->unlock();
    }
    return res;
}

double Vision_IPC::getTime()
{
    std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    return (timestamp*1.0e-9);
}

int64_t Vision_IPC::getTimeSeconds()
{
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int64_t Vision_IPC::getUpTimeSeconds()
{
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - m_startTime).count();
}

Vision_IPC::Error_e Vision_IPC::setDepth(Sensors_e id, Depth_t &newDepth)
{
    // qDebug() << " --" << this << "setDepth() --";
    Vision_IPC::Error_e error_ = Vision_IPC::Error_e::noError;
    if(shm_sensors_total > id) {
        QSharedMemory *_shm = shm_depth[id];
        if(_shm->size() == sizeof(Depth_t)) {
            if(_shm->lock())
            {
                // sync
                newDepth.tickWrite    = _getTickWrite(_shm);
                newDepth.tickRead     = _getTickRead(_shm);
                memcpy((char*)_shm->data(), (char *)&newDepth, _shm->size());
                // sync ticks
                {
                    _incrementTickWrite(_shm);
                    shm_depth_tickRead[id]     = _getTickRead(_shm);
                    shm_depth_tickWrite[id]    = _getTickWrite(_shm);
                }
                _shm->unlock();
            } else {
                error_ = Vision_IPC::Error_e::shmLockError;
            }
        } else {
            error_ = Vision_IPC::Error_e::sizeError;
        }
    } else {
        error_ = Vision_IPC::Error_e::idError;
    }
    return error_;
}

Vision_IPC::Error_e Vision_IPC::getDepth(Sensors_e id, Depth_t &outDepth)
{
    // qDebug() << " --" << this << "getDepth() --";
    Vision_IPC::Error_e error_ = Vision_IPC::Error_e::noError;
    if(shm_sensors_total > id) {
        QSharedMemory *_shm = shm_depth[id];
        if(_shm->size() == sizeof(Depth_t)) {
            if(_shm->lock())
            {
                memcpy((char*)&outDepth, (char*)_shm->constData(), _shm->size());
                // sync ticks
                {
                    _incrementTickRead(_shm);
                    shm_depth_tickRead[id]     = _getTickRead(_shm);
                    shm_depth_tickWrite[id]    = _getTickWrite(_shm);
                }
                _shm->unlock();
            } else {
                error_ = Vision_IPC::Error_e::shmLockError;
            }
        } else {
            error_ = Vision_IPC::Error_e::sizeError;
        }
    } else {
        error_ = Vision_IPC::Error_e::idError;
    }
    return error_;
}

Vision_IPC::Error_e Vision_IPC::setImageColor(Sensors_e id, ImageColor_t &newImageColor)
{
    // qDebug() << " --" << this << "setImageColor() begin --" << id;
    Vision_IPC::Error_e error_ = Vision_IPC::Error_e::noError;
    if(shm_sensors_total > id) {
        QSharedMemory *_shm = shm_color[id];
        if(_shm->size() == sizeof(ImageColor_t)) {
            if(_shm->lock())
            {
                // std::chrono::time_point<std::chrono::high_resolution_clock, long> time = std::chrono::high_resolution_clock::now();
                // sync
                newImageColor.tickWrite    = (_getTickWrite(_shm));
                newImageColor.tickRead     = _getTickRead(_shm);
                memcpy((char*)_shm->data(), (char *)&newImageColor, _shm->size());
                // sync ticks
                {
                    _incrementTickWrite(_shm);
                    shm_color_tickRead[id]     = _getTickRead(_shm);
                    shm_color_tickWrite[id]    = _getTickWrite(_shm);
                    // qDebug() << " --" << this << "setImageColor() tick write" << shm_color_tickWrite[id] << shm_color_tickRead[id];
                }
                _shm->unlock();
            } else {
                error_ = Vision_IPC::Error_e::shmLockError;
            }
        } else {
            error_ = Vision_IPC::Error_e::sizeError;
        }
    } else {
        error_ = Vision_IPC::Error_e::idError;
    }
    return error_;
}

Vision_IPC::Error_e Vision_IPC::getImageColor(Sensors_e id, ImageColor_t &outImageColor)
{
    // qDebug() << " --" << this << "getImageColor() --" << id;
    Vision_IPC::Error_e error_ = Vision_IPC::Error_e::noError;
    if(shm_sensors_total > id) {
        QSharedMemory *_shm = shm_color[id];
        if(_shm->size() == sizeof(ImageColor_t)) {
            if(_shm->lock())
            {
                memcpy((char*)&outImageColor, (char*)_shm->constData(), _shm->size());
                // sync ticks
                {
                    _incrementTickRead(_shm);
                    this->shm_color_tickRead[id]      = _getTickRead(_shm);
                    this->shm_color_tickWrite[id]     = _getTickWrite(_shm);
                }
                _shm->unlock();
            } else {
                error_ = Vision_IPC::Error_e::shmLockError;
            }
        } else {
            error_ = Vision_IPC::Error_e::sizeError;
        }
    } else {
        error_ = Vision_IPC::Error_e::idError;
    }
    return error_;
}

Vision_IPC::Error_e Vision_IPC::setImageIR(Sensors_e id, ImageIR_t &newImageIR)
{
    // qDebug() << " --" << this << "setImageIR() --";
    Vision_IPC::Error_e error_ = Vision_IPC::Error_e::noError;
    if(shm_sensors_total > id) {
        QSharedMemory *_shm = shm_ir[id];
        if(_shm->size() == sizeof(ImageIR_t)) {
            if(_shm->lock())
            {
                // std::chrono::time_point<std::chrono::high_resolution_clock, long> time = std::chrono::high_resolution_clock::now();
                // sync
                newImageIR.tickWrite    = _getTickWrite(_shm);
                newImageIR.tickRead     = _getTickRead(_shm);
                memcpy((char*)_shm->data(), (char *)&newImageIR, _shm->size());
                // sync ticks
                {
                    _incrementTickWrite(_shm);
                    shm_ir_tickRead[id]     = _getTickRead(_shm);
                    shm_ir_tickWrite[id]    = _getTickWrite(_shm);
                    // qDebug() << " --" << this << "setImageIR() tick write" << shm_ir_tickWrite[id] << shm_ir_tickRead[id];
                }
                _shm->unlock();
            } else {
                error_ = Vision_IPC::Error_e::shmLockError;
            }
        } else {
            error_ = Vision_IPC::Error_e::sizeError;
        }
    } else {
        error_ = Vision_IPC::Error_e::idError;
    }
    return error_;
}

Vision_IPC::Error_e Vision_IPC::getImageIR(Sensors_e id, ImageIR_t &outImageIR)
{
    // qDebug() << " --" << this << "getImageIR() --";
    Vision_IPC::Error_e error_ = Vision_IPC::Error_e::noError;
    if(shm_sensors_total > id) {
        QSharedMemory *_shm = shm_ir[id];
        if(_shm->size() == sizeof(ImageIR_t)) {
            if(_shm->lock())
            {
                memcpy((char*)&outImageIR, (char*)_shm->constData(), _shm->size());
                // sync ticks
                {
                    _incrementTickRead(_shm);
                    shm_ir_tickRead[id]     = _getTickRead(_shm);
                    shm_ir_tickWrite[id]    = _getTickWrite(_shm);
                }
                _shm->unlock();
            } else {
                error_ = Vision_IPC::Error_e::shmLockError;
            }
        } else {
            error_ = Vision_IPC::Error_e::sizeError;
        }
    } else {
        error_ = Vision_IPC::Error_e::idError;
    }
    return error_;
}
