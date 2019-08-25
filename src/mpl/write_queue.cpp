#include <jilog.hpp>
#include <system_error>
#include <mpl/write_queue.hpp>
#include <sys/errno.h>

void mpl::WriteQueue::writeTo(int socket) {
    if (empty())
        return;
    
    iovs_.clear();
    for (auto it = buffers_.begin() ; iovs_.size() < MAX_IOVS && it != buffers_.end() ; ++it) {
        iovs_.emplace_back();
        iovs_.back().iov_base = it->begin();
        iovs_.back().iov_len = it->remaining();
    }

    JI_LOG(TRACE) << "about to write " << iovs_.size() << " iovecs";
    ssize_t n = ::writev(socket, iovs_.data(), iovs_.size());
    JI_LOG(TRACE) << "wrote " << n;
    if (n == -1) {
        if (errno == EAGAIN)
            return;
        throw std::system_error(errno, std::system_category(), "writev");
    }

    while (n > 0) {
        if (n >= buffers_.front().remaining()) {
            n -= buffers_.front().remaining();
            buffers_.pop_front();
            JI_LOG(TRACE) << "removing completed buffer";
        } else {
            JI_LOG(TRACE) << "updating buffer in front";
            buffers_.front() += n;
            break;
        }
    }
}
