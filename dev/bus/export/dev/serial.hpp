//! \file
//! \brief Serial interface for platform drivers with listen mode support.
//! \todo Find better place (and probably name?) for current header and class.
#ifndef DEV_BUS_SERIAL_HPP_
#define DEV_BUS_SERIAL_HPP_

#include <ecl/err.hpp>
#include <ecl/assert.h>
#include <ecl/utils.hpp>
#include <common/bus.hpp>
#include <ecl/thread/semaphore.hpp>

#include <atomic>

namespace ecl
{

//! Serial driver interface.
//! \details The serial allows to abstract async,
//! interrupt-driven nature of platform-level drivers and
//! provide synchronus, buffered data management interface.
//! \tparam PBus Exclusively owned platform bus.
//! \tparam buf_size Size of internal rx and tx buffers.
template<class PBus, size_t buf_size = 128>
class serial
{
public:
    static constexpr auto buffer_size = buf_size;
    using platform_handle = PBus;

    serial() = delete;
    serial(const serial &other) = delete;
    serial(serial &&other) = delete;
    ~serial() = delete;

    //! Initialize serial driver and underlying platform bus.
    //! \pre Driver is not initialized.
    //! \return Operation status.
    static err init();

    //! Deinitialize serial driver and underlying platform bus.
    //! \pre Driver is initialized.
    //! \return Operation status.
    static err deinit();

    //! Obtains a byte from serial device.
    //! \pre Driver is initialized.
    //! \details Call will block if there is no data to return.
    //! Otherwise data will be stored in the given argument and
    //! method will return.
    //! \param[out] byte Place for incoming byte. Will remain
    //!                  unchanged if error occurs.
    //! \return Operation status.
    //! \retval err::nobufs Some data will be lost after the call completes.
    //!
    static err recv_byte(uint8_t &byte);

    //! Gets buffer from serial device.
    //! \pre Driver is initialized.
    //! \details Call will block if there is no data to return.
    //! Otherwise, buffer will be populated with avaliable data
    //! and method will return.
    //! \param[out]     buf Buffer to fill with data.
    //! \param[in,out]  sz  Size of buffer. Will be updated with
    //!                     amount of bytes written to the buffer.
    //!                     In case of error, size will be updated
    //!                     with bytes successfully stored in the buffer
    //!                     before error occurred.
    //! \return Operation status.
    //! \retval err::nobufs Some data will be lost after the call completes.
    static err recv_buf(uint8_t *buf, size_t &sz);

    //! Sends byte to a serial device.
    //! \pre Driver is initialized.
    //! \details It may not block if internal buffering is applied.
    //! It may also block if platform device is not yet ready to
    //! send data.
    //! \param[in] byte Byte to send.
    //! \return Operation status.
    static err send_byte(uint8_t byte);

    //! Sends buffer to a serial stream.
    //! \pre Driver is initialized.
    //! \details It may not block if internal buffering is applied.
    //! It may also block if platform device is not yet ready to
    //! send data.
    //! \param[in]      buf Buffer to send.
    //! \param[in,out]  sz  Size of buffer. Will be updated with
    //!                     amount of bytes sent In case of error,
    //!                     size will be updated with amount of bytes
    //!                     successfully sent before error occur.
    //! \return Operation status.
    //! \retval err:again The buffer was not consumed entirely.
    static err send_buf(const uint8_t *buf, size_t &sz);

private:
    static void bus_handler(bus_channel ch, bus_event type, size_t total);

    static bool m_is_inited;


// ---------------

    struct chunk
    {
        using bsem = safe_storage<binary_semaphore>;
        using asize_t = std::atomic_size_t;

        static constexpr auto xfer_size = buffer_size / 2;
        static constexpr auto size() { return xfer_size; }

        uint8_t   data[xfer_size];
        asize_t   start {0}; // Write by user, read by xfer and user
        asize_t   end {0}; // Write by xfer, read by xfer and user
        bool      xfer_pend = false;
        bsem      data_flag = {};

        chunk() { data_flag.init(); }

        // Called by user when buffer is finally depleted.
        void restore() { start = end = 0; }
        // Called by xfer, if next buffer still read by the user.
        void set_pend() { xfer_pend = true; }
        // Called by user after moving to the next buffer.
        bool pend() { return xfer_pend; }
        // Called by xfer to check if buffer is not used
        bool fresh() { return !start && !end; }

        // Called by user or xfer, when new xfer should be started
        // Pre: previous xfer must be completed
        ecl::err start_xfer()
        {
            PBus::set_rx(data, size());

            auto rc = PBus::enable_listen_mode();
            if (is_error(rc)) {
                PBus::set_rx(nullptr, 0);
                return rc;
            }

            rc = PBus::do_rx();
            if (is_error(rc)) {
                PBus::set_rx(nullptr, 0);
                return rc;
            }

            xfer_pend = false;
            return rc;
        }

        // Called by user to check how much data is there
        size_t fill() { ecl_assert(end >= start); return end - start; }
        // Called by user and xfer, to check if there are new data to read.
        bool no_data() { return !fill(); }
        bool depleted() { return start == size(); }
        bool no_space() { return end == size(); }

        bool wait_data() { data_flag.get().wait(); return true; } // TODO
        void signal_data() { data_flag.get().signal(); }

        void new_bytes(size_t cnt) { end += cnt; ecl_assert(end <= size()); }

        size_t copy(uint8_t *dst, size_t sz)
        {
            auto to_copy = std::min(sz, fill());
            ecl_assert(to_copy); // There must be some data.

            //std::cout << "copy -> " << start << " -> " << to_copy << std::endl;

            std::copy(data + start, data + start + to_copy, dst);
            start += to_copy;
            ecl_assert(start <= end);
            return to_copy;
        }

    };

    struct serial_chunks
    {
        chunk chunks[2] = {};
        int user_idx = 0;
        int xfer_idx = 0;

        chunk &user() { return chunks[user_idx]; }
        chunk &xfer() { return chunks[xfer_idx]; }

        chunk &user_next() { user_idx ^= 1; return user(); }
        chunk &xfer_next() { xfer_idx ^= 1; return xfer(); }
    };

    static safe_storage<serial_chunks> m_chunks;
    static safe_storage<binary_semaphore> m_tx_rdy; //!< Signalled if tx channel is ready
    static uint8_t m_tx_buf[buf_size];
};

template <class PBus, size_t buf_size>
bool serial<PBus, buf_size>::m_is_inited;

template <class PBus, size_t buf_size>
safe_storage<typename serial<PBus, buf_size>::serial_chunks> serial<PBus, buf_size>::m_chunks;

template <class PBus, size_t buf_size>
safe_storage<binary_semaphore> serial<PBus, buf_size>::m_tx_rdy;

template <class PBus, size_t buf_size>
uint8_t serial<PBus, buf_size>::m_tx_buf[buf_size];

template <class PBus, size_t buf_size>
err serial<PBus, buf_size>::init()
{
    m_chunks.init();
    ecl_assert(!m_is_inited);

    auto result = PBus::init();
    if (is_error(result)) {
        return result;
    }
    PBus::set_handler(bus_handler);
    PBus::set_tx(m_tx_buf, buffer_size);

    m_tx_rdy.get().signal();
    result = m_chunks.get().xfer().start_xfer();
    if (is_ok(result)) {
        m_is_inited = true;
    }
    return result;
}

template <class PBus, size_t buf_size>
err serial<PBus, buf_size>::deinit()
{
    ecl_assert(m_is_inited);
    PBus::cancel_xfer();
    PBus::reset_buffers();
    m_is_inited = false;
    m_tx_rdy.deinit();
    return err::ok;
}

template <class PBus, size_t buf_size>
void serial<PBus, buf_size>::bus_handler(bus_channel ch, bus_event type, size_t total)
{
    if (ch == bus_channel::rx) {
        if (type == bus_event::tc) {
            auto &xfer_buf = m_chunks.get().xfer();

            // TODO: correct assert to consume more than 1 byte at time.
            ecl_assert(xfer_buf.end + 1 == total);

            if (xfer_buf.no_data()) {
                xfer_buf.signal_data();
            }

            xfer_buf.new_bytes(1);

            if (xfer_buf.no_space()) {
                auto &new_xfer_buf = m_chunks.get().xfer_next();
                if (new_xfer_buf.fresh()) {
                    new_xfer_buf.start_xfer();
                } else {
                    new_xfer_buf.set_pend();
                }
            }
        }
    } else if (ch == bus_channel::tx) {
        if (type == bus_event::tc) {
            m_tx_rdy.get().signal();
        } // else error - ignore. TC event _must_ be supplied
          // after possible error.
    }
}

template <class PBus, size_t buf_size>
err serial<PBus, buf_size>::recv_byte(uint8_t &byte)
{
    size_t sz = 1;
    return recv_buf(&byte, sz);
}

template <class PBus, size_t buf_size>
err serial<PBus, buf_size>::recv_buf(uint8_t *buf, size_t &sz)
{
    ecl_assert(m_is_inited);
    err result = err::ok;

    auto &user_buf = m_chunks.get().user();

    while (user_buf.no_data()) { // TODO: reading xfer-owned variable
        user_buf.wait_data(); // All ok: semaphore
    }

    // Data is there, process...
    size_t read = user_buf.copy(buf, sz); // TODO: reading xfer-owned variable
    sz = read;

    // No data inside buffer.
    if (user_buf.depleted()) { // All ok: user-owned variable
        user_buf.restore(); // All ok: user-owned variable

        m_chunks.get().user_next();
        // Start xfer on buffer that was previously occupied by user.
        if (user_buf.pend()) { // TODO: reading xfer-owned variable
            user_buf.start_xfer();
        }
    }

    return result;
}

template <class PBus, size_t buf_size>
err serial<PBus, buf_size>::send_byte(uint8_t byte)
{
    size_t sz = 1;
    return send_buf(&byte, sz);
}

template <class PBus, size_t buf_size>
err serial<PBus, buf_size>::send_buf(const uint8_t *buf, size_t &size)
{
    // Wait until TX will be ready
    m_tx_rdy.get().wait();

    auto to_copy = std::min(size, buf_size);
    std::copy(buf, buf + to_copy, m_tx_buf);
    auto rc = PBus::do_tx();

    if (is_error(rc)) {
        return rc;
    }

    // Only part of the buffer was consumed
    if (to_copy < size) {
        rc = err::again;
    }

    size = to_copy;

    return rc;
}

} // namespace ecl

#endif // DEV_BUS_SERIAL_HPP_
