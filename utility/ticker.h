#ifndef TICKER_H
#define TICKER_H

template <typename Number>
class Ticker final {
   public:
    bool tick() {
        if (count_ == 0) {
            return false;
        }
        --count_;
        return true;
    }

    void reset(Number ticks) {
        count_ = ticks;
    }

   private:
    Number count_{0};
};

#endif  // TICKER_H
