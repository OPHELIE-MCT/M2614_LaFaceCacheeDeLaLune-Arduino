#include "FeedbackEncoder.h"

FeedbackEncoder* FeedbackEncoder::activeInstance_ = nullptr;

FeedbackEncoder::FeedbackEncoder(const FeedbackEncoderPins& pins)
    : pins_(pins), running_(false), threadId_(nullptr), currentSpeed{0, 0, 0, 0} {
    resetState();
}

FeedbackEncoder::~FeedbackEncoder() {
    end();
}

void FeedbackEncoder::begin() {
    if (running_) {
        return;
    }

    resetState();

    // Channel A is configured for full wiring compatibility, but not processed.
    pinMode(pins_.frontLeft.a, INPUT);
    pinMode(pins_.frontRight.a, INPUT);
    pinMode(pins_.rearLeft.a, INPUT);
    pinMode(pins_.rearRight.a, INPUT);

    pinMode(pins_.frontLeft.b, INPUT);
    pinMode(pins_.frontRight.b, INPUT);
    pinMode(pins_.rearLeft.b, INPUT);
    pinMode(pins_.rearRight.b, INPUT);

    activeInstance_ = this;

    attachInterrupt(digitalPinToInterrupt(pins_.frontLeft.b), isrFrontLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins_.frontRight.b), isrFrontRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins_.rearLeft.b), isrRearLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins_.rearRight.b), isrRearRight, CHANGE);

    running_ = true;
    threadId_ = k_thread_create(&threadData_, threadStack_, sizeof(threadStack_), processingThreadEntry, this, nullptr, nullptr, kThreadPriority, 0, K_NO_WAIT);
}

void FeedbackEncoder::end() {
    if (!running_) {
        return;
    }

    running_ = false;

    if (threadId_ != nullptr) {
        k_thread_abort(threadId_);
        threadId_ = nullptr;
    }

    detachInterrupt(digitalPinToInterrupt(pins_.frontLeft.b));
    detachInterrupt(digitalPinToInterrupt(pins_.frontRight.b));
    detachInterrupt(digitalPinToInterrupt(pins_.rearLeft.b));
    detachInterrupt(digitalPinToInterrupt(pins_.rearRight.b));

    if (activeInstance_ == this) {
        activeInstance_ = nullptr;
    }
}

EncoderSpeedSnapshot FeedbackEncoder::getCurrentSpeed() const {
    return currentSpeed;
}

void FeedbackEncoder::resetState() {
    for (uint8_t i = 0; i < kWheelCount; ++i) {
        atomic_set(&edgeCounters_[i], 0);
    }
    currentSpeed.frontLeft = 0;
    currentSpeed.frontRight = 0;
    currentSpeed.rearLeft = 0;
    currentSpeed.rearRight = 0;
}

void FeedbackEncoder::onChannelBEdge(WheelIndex wheel) {
    atomic_inc(&edgeCounters_[wheel]);
}

void FeedbackEncoder::processCounters() {
    currentSpeed.frontLeft = atomic_set(&edgeCounters_[kFrontLeft], 0);
    currentSpeed.frontRight = atomic_set(&edgeCounters_[kFrontRight], 0);
    currentSpeed.rearLeft = atomic_set(&edgeCounters_[kRearLeft], 0);
    currentSpeed.rearRight = atomic_set(&edgeCounters_[kRearRight], 0);
}

void FeedbackEncoder::processingThreadEntry(void* p1, void* p2, void* p3) {
    (void)p2;
    (void)p3;

    FeedbackEncoder* self = static_cast<FeedbackEncoder*>(p1);
    if (self == nullptr) {
        return;
    }

    while (self->running_) {
        self->processCounters();
        k_msleep(kThreadPeriodMs);
    }
}

void FeedbackEncoder::isrFrontLeft() {
    if (activeInstance_ != nullptr) {
        activeInstance_->onChannelBEdge(kFrontLeft);
    }
}

void FeedbackEncoder::isrFrontRight() {
    if (activeInstance_ != nullptr) {
        activeInstance_->onChannelBEdge(kFrontRight);
    }
}

void FeedbackEncoder::isrRearLeft() {
    if (activeInstance_ != nullptr) {
        activeInstance_->onChannelBEdge(kRearLeft);
    }
}

void FeedbackEncoder::isrRearRight() {
    if (activeInstance_ != nullptr) {
        activeInstance_->onChannelBEdge(kRearRight);
    }
}
