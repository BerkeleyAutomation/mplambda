#pragma once
#ifndef MPL_BUFFER_HPP
#define MPL_BUFFER_HPP

#include <vector>
#include <cassert>

namespace mpl {
    class Buffer {
        char *base_;
        char *position_;
        char *limit_;
        std::size_t capacity_;
        
    public:
        Buffer()
            : base_{nullptr}
            , position_{nullptr}
            , limit_{nullptr}
            , capacity_{0}
        {
        }

        explicit Buffer(std::size_t n)
            : base_{new char[n]}
            , position_{base_}
            , limit_{base_ + n}
            , capacity_{n}
        {
        }

        Buffer(const Buffer&) = delete;
        Buffer(Buffer&& other)
            : base_{std::exchange(other.base_, nullptr)}
            , position_{std::exchange(other.position_, nullptr)}
            , limit_{std::exchange(other.limit_, nullptr)}
            , capacity_{std::exchange(other.capacity_, 0)}
        {
        }

        ~Buffer() {
            delete[] base_;
        }

        Buffer& operator = (const Buffer&) = delete;
        Buffer& operator = (Buffer&& other) {
            std::swap(base_, other.base_);
            std::swap(position_, other.position_);
            std::swap(limit_, other.limit_);
            std::swap(capacity_, other.capacity_);
            return *this;
        }

        char* begin() {
            return position_;
        }
        
        const char* begin() const {
            return position_;
        }

        char* end() {
            return limit_;
        }

        const char *end() const {
            return limit_;
        }

        std::size_t remaining() const {
            return std::distance(position_, limit_);
        }

        Buffer& operator += (int i) {
            position_ += i;
            assert(position_ <= limit_);
            return *this;
        }

        Buffer& flip() {
            limit_ = position_;
            position_ = base_;
            return *this;
        }

        Buffer& compact() {
            if (position_ != base_)
                std::copy(position_, limit_, base_);
            position_ = base_ + remaining();
            limit_ = base_ + capacity_;
            return *this;
        }

        template <class T>
        T peek(int offset = 0) const {
            T value;
            const char *p = begin() + offset;
            if (__ORDER_BIG_ENDIAN__ == __BYTE_ORDER__) {
                std::copy(p, p+sizeof(T), reinterpret_cast<char*>(&value));
            } else {
                std::reverse_copy(p, p+sizeof(T), reinterpret_cast<char*>(&value));
            }
            return value;
        }

        template <class T>
        void put(const T& value) {
            assert(position_ + sizeof(T) <= limit_);
            const char *p = reinterpret_cast<const char *>(&value);
            if (__ORDER_BIG_ENDIAN__ == __BYTE_ORDER__) {
                std::copy(p, p+sizeof(T), position_);
            } else {
                std::reverse_copy(p, p+sizeof(T), position_);
            }
            position_ += sizeof(T);
        }
    };
}

#endif
