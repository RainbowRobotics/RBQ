#pragma once

// CDR (XCDR1, little endian) for the IDL-generated types — the exact bytes rmw_zenoh / Fast-CDR put on the wire:
// a 4-byte encapsulation header (00 01 00 00 = CDR_LE) followed by classic CDR, 8-byte aligned relative to the
// data after the header. Generated types are FINAL, so XCDR1 has no member headers.

#include <cstdint>
#include <cstring>
#include <vector>

#include <org/eclipse/cyclonedds/core/cdr/basic_cdr_ser.hpp>

namespace rbq_sdk::zn::cdr {

constexpr uint8_t kHeaderLE[4] = {0x00, 0x01, 0x00, 0x00};

// The generated move/write/read overloads live in org::eclipse::cyclonedds::core::cdr next to the stream type.
// They are called unqualified so argument-dependent lookup finds them where the template is instantiated — a
// qualified call would only see overloads declared before this header, i.e. depend on include order.

/// Payload size of msg (header included). Returns false if the type can't be serialized (e.g. a bounded field
/// over its bound).
template <typename T>
bool serializedSize(const T& msg, size_t& size)
{
    namespace c = org::eclipse::cyclonedds::core::cdr;
    c::basic_cdr_stream sizer(c::endianness::little_endian);
    if (!move(sizer, msg, false)) return false;
    size = 4 + sizer.position();
    return true;
}

/// Serialize msg into buf of exactly serializedSize(msg) bytes. Every byte is written, alignment padding included
/// (the writer zeroes it), so buf need not be initialised — a 6 MB camera frame is not memset first.
template <typename T>
bool serializeInto(const T& msg, uint8_t* buf, size_t size)
{
    namespace c = org::eclipse::cyclonedds::core::cdr;
    std::memcpy(buf, kHeaderLE, 4);
    c::basic_cdr_stream writer(c::endianness::little_endian);
    writer.set_buffer(buf + 4, size - 4);
    return write(writer, msg, false);
}

template <typename T>
bool serialize(const T& msg, std::vector<uint8_t>& out)
{
    size_t n = 0;
    if (!serializedSize(msg, n)) return false;
    out.resize(n);
    return serializeInto(msg, out.data(), n);
}

/// Deserialize a payload (header included) into msg. Accepts both endiannesses.
template <typename T>
bool deserialize(const uint8_t* data, size_t size, T& msg)
{
    namespace c = org::eclipse::cyclonedds::core::cdr;
    if (size < 4 || data[0] != 0x00 || (data[1] != 0x00 && data[1] != 0x01)) return false;
    const auto end = data[1] == 0x01 ? c::endianness::little_endian : c::endianness::big_endian;
    c::basic_cdr_stream reader(end);
    // The stream API takes a mutable buffer; read mode never writes to it.
    reader.set_buffer(const_cast<uint8_t*>(data) + 4, size - 4);
    return read(reader, msg, false);
}

} // namespace rbq_sdk::zn::cdr
