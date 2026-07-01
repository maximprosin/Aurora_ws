#ifndef UDP_PROTOCOL_H
#define UDP_PROTOCOL_H

#include <cstdint>   // для uint32_t, uint16_t, uint8_t
#include <cstddef>   // для size_t
#include <vector>    // для std::vector

namespace udp_protocol {

// ============================================================
// КОНСТАНТЫ
// ============================================================
constexpr uint32_t MAGIC_NUMBER = 0xDEADBEEF;
constexpr size_t MAX_UDP_PACKET_SIZE = 65507;  // Максимальный размер UDP пакета
constexpr size_t HEADER_SIZE = 24;              // Размер заголовка

// ============================================================
// ЗАГОЛОВОК ПАКЕТА
// ============================================================
#pragma pack(push, 1)
struct PacketHeader {
    uint32_t magic;           // 0xDEADBEEF
    uint16_t packet_id;       // ID пакета в последовательности
    uint16_t total_packets;   // Всего пакетов в кадре
    uint32_t timestamp;       // Временная метка (мс)
    uint16_t width;           // Ширина изображения
    uint16_t height;          // Высота изображения
    uint16_t data_size;       // Размер данных в этом пакете
    uint8_t  format;          // 0 = JPEG, 1 = H.265, 2 = RAW_BGR
    uint8_t  flags;           // Битовые флаги

    // Флаги:
    // bit 0: первый пакет кадра (START)
    // bit 1: последний пакет кадра (END)
    // bit 2: ключевой кадр (I-frame)
};
#pragma pack(pop)

// ============================================================
// РАЗМЕР ПОЛЕЗНЫХ ДАННЫХ В ПАКЕТЕ
// ============================================================
constexpr size_t PAYLOAD_SIZE = MAX_UDP_PACKET_SIZE - HEADER_SIZE;

// ============================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================

// Создание заголовка
inline PacketHeader create_header(
    uint16_t packet_id,
    uint16_t total_packets,
    uint32_t timestamp,
    uint16_t width,
    uint16_t height,
    uint16_t data_size,
    uint8_t format,
    uint8_t flags = 0
    ) {
    PacketHeader header;
    header.magic = MAGIC_NUMBER;
    header.packet_id = packet_id;
    header.total_packets = total_packets;
    header.timestamp = timestamp;
    header.width = width;
    header.height = height;
    header.data_size = data_size;
    header.format = format;
    header.flags = flags;
    return header;
}

// Проверка заголовка
inline bool is_valid_header(const PacketHeader& header) {
    return header.magic == MAGIC_NUMBER;
}

// Проверка, что пакет - первый в кадре
inline bool is_first_packet(const PacketHeader& header) {
    return (header.flags & 0x01) != 0;
}

// Проверка, что пакет - последний в кадре
inline bool is_last_packet(const PacketHeader& header) {
    return (header.flags & 0x02) != 0;
}

// Проверка, что кадр ключевой
inline bool is_keyframe(const PacketHeader& header) {
    return (header.flags & 0x04) != 0;
}

} // namespace udp_protocol

#endif // UDP_PROTOCOL_H