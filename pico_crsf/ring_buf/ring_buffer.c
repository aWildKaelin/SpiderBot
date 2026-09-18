#include "ring_buffer.h"


bool ringBuffer_init(ringBuffer *buff, size_t size){
    buff->buffer = (unsigned char*)malloc(size);

    if(buff->buffer == NULL)
        return false;

    buff->capacity = size;
    buff->readHead = 0;
    buff->writeHead = 0;
    buff->isFull = false;

    return true;
}

void ringBuffer_free(ringBuffer *buff){
    free(buff->buffer);
}

bool ringBuffer_writeChar(ringBuffer *buff, unsigned char a){
    if(buff->writeHead == buff->readHead && buff->isFull)
        return false;
    
    buff->buffer[buff->writeHead++] = a;
    if(buff->writeHead == buff->capacity)
        buff->writeHead = 0;

    if(buff->writeHead == buff->readHead)
        buff->isFull = true;
    
    return true;
}

int ringBuffer_readChar(ringBuffer *buff){
    if(buff->readHead == buff->writeHead && !buff->isFull)
        return -1;

    size_t pos = buff->readHead++;

    if(buff->readHead == buff->capacity)
        buff->readHead = 0;
    
    buff->isFull = false;
    return buff->buffer[pos];
}


bool ringBuffer_isEmpty(ringBuffer *buff){
    return (!buff->isFull && buff->readHead == buff->writeHead);
}

bool ringBuffer_isFull(ringBuffer *buff){
    return buff->isFull;
}

size_t ringBuffer_getContentSize(ringBuffer *buff){
    if(ringBuffer_isFull(buff)) return buff->capacity;
    else if(ringBuffer_isEmpty(buff)) return 0;
    else if(buff->readHead < buff->writeHead)
        return buff->writeHead - buff->readHead;
    else 
        return buff->capacity - (buff->writeHead - buff->readHead);
}