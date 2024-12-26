// thread_safe_queue.hpp
#pragma once

#include <array>
#include <optional>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

template<typename T, size_t SIZE = 10>
class thread_safe_queue {
public:
    thread_safe_queue() {
        mutex_ = xSemaphoreCreateMutex();
        if (!mutex_) {
            ESP_LOGE("Queue", "Failed to create mutex");
            return;
        }
        
        // Binary semaphore for signaling new data
        dataAvailable_ = xSemaphoreCreateBinary();
        if (!dataAvailable_) {
            ESP_LOGE("Queue", "Failed to create semaphore");
            vSemaphoreDelete(mutex_);
            mutex_ = nullptr;
            return;
        }
    }

    ~thread_safe_queue() {
        if (mutex_) {
            vSemaphoreDelete(mutex_);
        }
        if (dataAvailable_) {
            vSemaphoreDelete(dataAvailable_);
        }
    }

    // Non-blocking push operation
    bool push(const T& item) {
        if (!mutex_) return false;

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return false;
        }

        bool success = false;
        if (count_ < SIZE) {
            buffer_[writeIndex_] = item;
            writeIndex_ = (writeIndex_ + 1) % SIZE;
            count_++;
            success = true;
            xSemaphoreGive(dataAvailable_); // Signal that new data is available
        }

        xSemaphoreGive(mutex_);
        return success;
    }

    // Blocking pop operation with timeout
    std::optional<T> pop(TickType_t waitTicks = portMAX_DELAY) {
        if (!mutex_ || !dataAvailable_) return std::nullopt;

        // Wait for data to be available
        if (xSemaphoreTake(dataAvailable_, waitTicks) != pdTRUE) {
            return std::nullopt;
        }

        if (xSemaphoreTake(mutex_, waitTicks) != pdTRUE) {
            xSemaphoreGive(dataAvailable_); // Return the semaphore
            return std::nullopt;
        }

        T item = buffer_[readIndex_];
        readIndex_ = (readIndex_ + 1) % SIZE;
        count_--;

        xSemaphoreGive(mutex_);
        return std::optional<T>(item);
    }

    // Non-blocking peek operation
    std::optional<T> peek() {
        if (!mutex_) return std::nullopt;

        if (xSemaphoreTake(mutex_, 0) != pdTRUE) {
            return std::nullopt;
        }

        std::optional<T> result;
        if (count_ > 0) {
            result = buffer_[readIndex_];
        }

        xSemaphoreGive(mutex_);
        return result;
    }

    // Get current number of items in queue
    size_t count() {
        if (!mutex_) return 0;

        if (xSemaphoreTake(mutex_, 0) != pdTRUE) {
            return 0;
        }

        size_t currentCount = count_;
        xSemaphoreGive(mutex_);
        return currentCount;
    }

    // Check if queue is empty
    bool empty() {
        return count() == 0;
    }

    // Check if queue is full
    bool full() {
        return count() == SIZE;
    }

    // Clear the queue
    void clear() {
        if (!mutex_) return;

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return;
        }

        readIndex_ = 0;
        writeIndex_ = 0;
        count_ = 0;

        xSemaphoreGive(mutex_);
    }

private:
    std::array<T, SIZE> buffer_;
    size_t readIndex_ = 0;
    size_t writeIndex_ = 0;
    size_t count_ = 0;
    SemaphoreHandle_t mutex_ = nullptr;
    SemaphoreHandle_t dataAvailable_ = nullptr;
};