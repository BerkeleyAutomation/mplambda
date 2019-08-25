#pragma once
#ifndef MPL_PACKET_HPP
#define MPL_PACKET_HPP

#include "buffer.hpp"

namespace mpl::packet {

    using Type = std::uint32_t;
    
    static constexpr Type HELLO = 0x3864caca;
    
    class Hello {
        std::uint64_t id_;
        
    public:
        explicit Hello(std::uint64_t id)
            : id_(id)
        {
        }
        
        operator Buffer () const {
            Buffer buf{16};
            buf.put(HELLO);
            buf.put(std::uint32_t(16));
            buf.put(id_);
            buf.flip();
            return buf;
        }
    };

}

#endif
