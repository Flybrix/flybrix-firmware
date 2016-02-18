#ifndef COBS_H
#define COBS_H

#include <cstddef>
#include <cstdint>
#include <vector>

template <class T, class... Targs>
constexpr std::size_t cobsPayloadSize(T&& t) {
    return sizeof(t);
}

template <class T, class... Targs>
constexpr std::size_t cobsPayloadSize(T&& t, Targs&&... targs) {
    return cobsPayloadSize(t) + cobsPayloadSize(targs...);
}

constexpr std::size_t packageFromPayloadSize(size_t N) {
    // assumes parity
    // adds up payload size (N), parity (1), trailing 0 (1), and optional extras (ceil(N/254))
    return N + 2 + (N + 253) / 254;
}

std::size_t cobsEncode(uint8_t* dst_ptr, const uint8_t* src_begin, const uint8_t* src_end);
std::size_t cobsDecode(uint8_t* dst_ptr, const uint8_t* src_ptr);

template <std::size_t N>
class CobsReader final {
   public:
    void AppendToBuffer(char c) {
        if (done) {
            // first byte of a new message
            done = false;
            buffer_length = 0;
        }

        buffer[buffer_length++] = c;

        if (c && buffer_length == N) {
            // buffer overflow, probably due to errors in data
            buffer_length = 0;
            return;
        }

        if (!c) {
            buffer_length = cobsDecode(buffer, buffer);
            if (!buffer_length)
                buffer[0] = 1;
            for (int i = 1; i < buffer_length; ++i)
                buffer[0] ^= buffer[i];
            // It is done only if the check results in a zero
            if (!buffer[0]) {
                output_start = 1;
                done = true;
            }
        }
    }

    template <class T>
    bool ParseInto(T& data) {
        if (!CanContain<T>())
            return false;
        uint8_t* v_begin{(uint8_t*)&data};
        uint8_t* v_end{v_begin + sizeof(T)};
        const uint8_t* buffer_pointer{buffer + output_start};
        output_start += sizeof(T);
        while (v_begin < v_end)
            *v_begin++ = *buffer_pointer++;
        return true;
    }

    template <class T, class... Targs>
    bool ParseInto(T& data, Targs&... data_args) {
        if (!ParseInto(data))
            return false;
        return ParseInto(data_args...);
    }

    template <class T>
    bool CanContain() const {
        if (!done)
            return false;
        if (sizeof(T) > buffer_length - output_start)
            return false;
        return true;
    }

    bool IsDone() const {
        return done;
    }

   private:
    uint8_t buffer[N];
    std::size_t output_start{1};
    std::size_t buffer_length{0};
    bool done{false};
};

template <std::size_t N>
struct CobsPackage final {
    uint8_t data[packageFromPayloadSize(N)];
    std::size_t length;
};

template <std::size_t N>
class CobsPayload final {
   public:
    template <class T>
    void Append(T&& v) {
        uint8_t* v_begin{(uint8_t*)&v};
        uint8_t* v_end{v_begin + sizeof(T)};
        while (v_begin != v_end)
            data[0] ^= data[l++] = *v_begin++;
        data[l] = 0;
    }

    template <class T, class... Targs>
    void Append(T&& v, Targs&&... vargs) {
        Append(v);
        Append(vargs...);
    }

    void ZeroPad(size_t desired_length = N) {
        while (l <= desired_length)
            data[++l] = 0;
    }

    inline std::size_t length() const {
        return l - 1;
    }

    CobsPackage<N> Encode() const {
        CobsPackage<N> pkg;
        pkg.length = cobsEncode(pkg.data, data, data + l + 1);  // include trailing zero
        return pkg;
    }

   private:
    uint8_t data[N + 2]{0};  // add trailing zero and leading checksum
    std::size_t l{1};
};

#endif
