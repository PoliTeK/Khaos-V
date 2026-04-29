#pragma once

#include <atomic>
#include <optional>

template <typename T> class TriBuf {
private:
    enum class Permutation : uint8_t { P123, P132, P213, P231, P312, P321 };

    T _data1, _data2, _data3;
    std::atomic<Permutation> _perm{TriBuf::Permutation::P123};
    std::atomic<bool> _has_writer{false}, _has_reader{false}, _middle_ready{false};

public:
    class Writer {
        friend class TriBuf<T>;

    private:
        TriBuf *_buf;
        Writer(TriBuf &buf) : _buf(&buf) {}

    public:
        Writer() : _buf(nullptr) {}
        Writer(const Writer &) = delete;

        Writer(Writer &&that) {
            _buf = that._buf;
            that._buf = nullptr;
        }

        Writer& operator=(Writer that) {
            std::swap(*this, that);
            return *this;
        }

        ~Writer() {
            if (_buf)
                _buf->_has_writer.store(false, std::memory_order_release);
        }

        T &data() {
            // exploits the fact that the reader will only swap the last two elements,
            // leaving the first intact
            auto perm = _buf->_perm.load(std::memory_order_acquire);

            switch (perm) {
            case TriBuf::Permutation::P123:
            case TriBuf::Permutation::P132:
                return _buf->_data1;
            case TriBuf::Permutation::P213:
            case TriBuf::Permutation::P231:
                return _buf->_data2;
            case TriBuf::Permutation::P312:
            case TriBuf::Permutation::P321:
            default:
                return _buf->_data3;
            }
        }

        const T &data() const { return const_cast<Writer *>(this)->data(); }

        void swap() {
            Permutation current = _buf->_perm.load(std::memory_order_acquire);
            Permutation desired;

            do {
                switch (current) {
                case Permutation::P123:
                    desired = Permutation::P213;
                    break;
                case Permutation::P132:
                    desired = Permutation::P312;
                    break;
                case Permutation::P213:
                    desired = Permutation::P123;
                    break;
                case Permutation::P231:
                    desired = Permutation::P321;
                    break;
                case Permutation::P312:
                    desired = Permutation::P132;
                    break;
                case Permutation::P321:
                default:
                    desired = Permutation::P231;
                    break;
                }
            } while (!_buf->_perm.compare_exchange_weak(current, desired,
                std::memory_order_release,
                std::memory_order_acquire));

            // signal to the reader that the middle data is ready
            // release: guarantees that on acquiring _middle_ready, the reader will
            // also see the updated permutation
            _buf->_middle_ready.store(true, std::memory_order_release);
        }
    };

    class Reader {
        friend class TriBuf<T>;

    private:
        TriBuf *_buf;
        Reader(TriBuf &buf) : _buf(&buf) {}

    public:
        Reader() : _buf(nullptr) {}
        Reader(const Reader &) = delete;

        Reader(Reader &&that) {
            _buf = that._buf;
            that._buf = nullptr;
        }

        Reader& operator=(Reader that) {
            std::swap(*this, that);
            return *this;
        }

        ~Reader() {
            if (_buf)
                _buf->_has_reader.store(false, std::memory_order_release);
        }

        const T &data() {
            // exploits the fact that the writer will only swap the first two
            // elements, leaving the last intact
            auto perm = _buf->_perm.load(std::memory_order_acquire);

            switch (perm) {
            case TriBuf::Permutation::P231:
            case TriBuf::Permutation::P321:
                return _buf->_data1;
            case TriBuf::Permutation::P132:
            case TriBuf::Permutation::P312:
                return _buf->_data2;
            case TriBuf::Permutation::P123:
            case TriBuf::Permutation::P213:
            default:
                return _buf->_data3;
            }
        }

        bool try_swap() {
            bool expected_ready = true;

            if (_buf->_middle_ready.compare_exchange_strong(
                            expected_ready, false, std::memory_order_acq_rel,
                            std::memory_order_relaxed)) {
                Permutation current = _buf->_perm.load(std::memory_order_acquire);
                Permutation desired;
                do {
                    switch (current) {
                    case Permutation::P123:
                        desired = Permutation::P132;
                        break;
                    case Permutation::P132:
                        desired = Permutation::P123;
                        break;
                    case Permutation::P213:
                        desired = Permutation::P231;
                        break;
                    case Permutation::P231:
                        desired = Permutation::P213;
                        break;
                    case Permutation::P312:
                        desired = Permutation::P321;
                        break;
                    case Permutation::P321:
                    default:
                        desired = Permutation::P312;
                        break;
                    }
                } while (!_buf->_perm.compare_exchange_weak(current, desired,
                    std::memory_order_release,
                    std::memory_order_acquire));
                return true;
            } else {
                return false;
            }
        }
    };

    TriBuf() : _data1(), _data2(), _data3() {}

    TriBuf(const T &data) : _data1(data), _data2(data), _data3(data) {}

    TriBuf(const T &data_wr, const T &data_rd)
        : _data1(data_wr), _data2(), _data3(data_rd) {}

    std::optional<Writer> get_writer() {
        bool expected = false;
        // success:
        //   - acquire data written by previous writers and permutation
        // failure:
        //   - relaxed -> do nothing
        if (_has_writer.compare_exchange_strong(expected, true,
            std::memory_order_acquire,
            std::memory_order_relaxed
        )) {
            return Writer{*this};
        } else {
            return {};
        }
    }

    std::optional<Reader> get_reader() {
        bool expected = false;
        // success:
        //   - acquire data written by previous writers and permutation
        // failure:
        //   - relaxed -> do nothing
        if (_has_reader.compare_exchange_strong(expected, true,
            std::memory_order_acquire,
            std::memory_order_relaxed
        )) {
            return Reader{*this};
        } else {
            return {};
        }
    }
};
