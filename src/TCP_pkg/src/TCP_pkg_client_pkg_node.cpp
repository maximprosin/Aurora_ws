#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <map>  // ДОБАВЛЕНО!

// ============================================
// НАСТРОЙКИ ПОДКЛЮЧЕНИЯ
// ============================================
const std::string SERVER_IP = "127.0.0.1";  // IP сервера
const int SERVER_PORT = 5005;                // Порт сервера

std::atomic<bool> running(true);
int sockfd = -1;

// ============================================
// ФУНКЦИЯ ДЛЯ ФОРМАТИРОВАННОГО ВЫВОДА
// ============================================
void print_separator(const std::string& title) {
    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << title << std::endl;
    std::cout << std::string(50, '=') << std::endl;
}

// ============================================
// ПОТОК ЧТЕНИЯ ОТ СЕРВЕРА
// ============================================
void server_read_thread() {
    char buffer[4096];
    std::string partial_line;

    print_separator("ОЖИДАНИЕ ДАННЫХ ОТ СЕРВЕРА");
    std::cout << "📡 Клиент подключен и слушает сервер..." << std::endl;
    std::cout << "   Все команды управления выполняются на сервере" << std::endl;
    std::cout << "   Здесь отображаются состояние и события\n" << std::endl;

    while (running && sockfd != -1) {
        memset(buffer, 0, sizeof(buffer));
        ssize_t bytes_read = recv(sockfd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string data = partial_line + std::string(buffer);
            partial_line.clear();

            // Разделяем на строки
            std::istringstream stream(data);
            std::string line;

            while (std::getline(stream, line)) {
                // Удаляем \r
                line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

                if (!line.empty()) {
                    // Форматированный вывод в зависимости от типа сообщения
                    if (line.find("CAMERA:") == 0) {
                        // Данные камеры: CAMERA:pitch,yaw
                        std::string values = line.substr(7);
                        size_t comma = values.find(',');
                        if (comma != std::string::npos) {
                            std::string pitch_str = values.substr(0, comma);
                            std::string yaw_str = values.substr(comma + 1);

                            std::cout << "📷 КАМЕРА | Наклон: " << std::setw(7) << pitch_str
                                      << "° | Поворот: " << std::setw(7) << yaw_str << "°" << std::endl;
                        } else {
                            std::cout << "📷 " << line << std::endl;
                        }
                    }
                    else if (line.find("TELEOP:") == 0) {
                        // Данные тележки: TELEOP:linear,angular
                        std::string values = line.substr(7);
                        size_t comma = values.find(',');
                        if (comma != std::string::npos) {
                            std::string linear_str = values.substr(0, comma);
                            std::string angular_str = values.substr(comma + 1);

                            std::cout << "🚜 ТЕЛЕЖКА | Скорость: " << std::setw(7) << linear_str
                                      << " | Поворот: " << std::setw(7) << angular_str << std::endl;
                        } else {
                            std::cout << "🚜 " << line << std::endl;
                        }
                    }
                    else if (line.find("KEY:") == 0) {
                        // Нажатия клавиш
                        std::string key_info = line.substr(4);

                        // Маппинг клавиш для красивого отображения
                        std::map<std::string, std::string> key_display;
                        key_display["W:press"] = "🚜 ВПЕРЁД";
                        key_display["S:press"] = "🚜 НАЗАД";
                        key_display["A:press"] = "🚜 НАЛЕВО";
                        key_display["D:press"] = "🚜 НАПРАВО";
                        key_display["Q:press"] = "🚜 КРУИЗ-КОНТРОЛЬ";
                        key_display["R:press"] = "🚜 СКОРОСТЬ +";
                        key_display["F:press"] = "🚜 СКОРОСТЬ -";
                        key_display["SPACE:press"] = "🚜 СТОП";
                        key_display["UP:press"] = "📷 НАКЛОН ВВЕРХ";
                        key_display["DOWN:press"] = "📷 НАКЛОН ВНИЗ";
                        key_display["LEFT:press"] = "📷 ПОВОРОТ НАЛЕВО";
                        key_display["RIGHT:press"] = "📷 ПОВОРОТ НАПРАВО";
                        key_display["PLUS:press"] = "📷 ПРИБЛИЖЕНИЕ";
                        key_display["MINUS:press"] = "📷 ОТДАЛЕНИЕ";
                        key_display["Z:press"] = "📷 ЗУМ +";
                        key_display["X:press"] = "📷 ЗУМ -";
                        key_display["C:press"] = "📷 СТОП ЗУМ";
                        key_display["W:release"] = "🚜 ОТПУСТИЛИ ВПЕРЁД";
                        key_display["S:release"] = "🚜 ОТПУСТИЛИ НАЗАД";
                        key_display["A:release"] = "🚜 ОТПУСТИЛИ НАЛЕВО";
                        key_display["D:release"] = "🚜 ОТПУСТИЛИ НАПРАВО";
                        key_display["SPACE:release"] = "🚜 ОТПУСТИЛИ СТОП";

                        auto it = key_display.find(key_info);
                        if (it != key_display.end()) {
                            std::cout << "⌨️  КЛАВИША | " << it->second << std::endl;
                        } else {
                            std::cout << "⌨️  " << line << std::endl;
                        }
                    }
                    else if (line.find("KEYBOARD:") == 0) {
                        std::string kb_info = line.substr(9);
                        std::cout << "⌨️  КОМАНДА | " << kb_info << std::endl;
                    }
                    else if (line.find("Connected to ROS 2 Bridge") != std::string::npos) {
                        std::cout << "✅ " << line << std::endl;
                    }
                    else if (line.find("УПРАВЛЕНИЕ") != std::string::npos ||
                             line.find("ТЕЛЕЖКОЙ") != std::string::npos ||
                             line.find("КАМЕРОЙ") != std::string::npos) {
                        std::cout << "   " << line << std::endl;
                    }
                    else if (line.find("===") != std::string::npos) {
                        std::cout << line << std::endl;
                    }
                    else {
                        std::cout << "📨 " << line << std::endl;
                    }
                }
            }
        }
        else if (bytes_read == 0) {
            std::cout << "\n❌ Сервер отключился" << std::endl;
            running = false;
            break;
        }
        else {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "❌ Ошибка чтения: " << strerror(errno) << std::endl;
                running = false;
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ============================================
// ПОТОК ОЖИДАНИЯ КОМАНДЫ ВЫХОДА
// ============================================
void exit_wait_thread() {
    std::cout << "\n💡 Нажмите ENTER для выхода из клиента\n" << std::endl;
    std::cin.get();
    running = false;
}

// ============================================
// ГЛАВНАЯ ФУНКЦИЯ
// ============================================
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   TCP КЛИЕНТ МОНИТОРИНГА РОБОТА" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Подключение к " << SERVER_IP << ":" << SERVER_PORT << "..." << std::endl;

    // Создаём сокет
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cerr << "❌ Не удалось создать сокет: " << strerror(errno) << std::endl;
        return 1;
    }

    // Настраиваем адрес сервера
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);

    if (inet_pton(AF_INET, SERVER_IP.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "❌ Неверный IP-адрес: " << SERVER_IP << std::endl;
        close(sockfd);
        return 1;
    }

    // Подключаемся
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "❌ Не удалось подключиться: " << strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    // Устанавливаем неблокирующий режим для сокета
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    }

    std::cout << "✅ Подключено к серверу!" << std::endl;

    // Запускаем потоки
    std::thread read_thread(server_read_thread);
    std::thread exit_thread(exit_wait_thread);

    // Ждём завершения
    read_thread.join();
    exit_thread.join();

    // Закрываем сокет
    if (sockfd != -1) {
        close(sockfd);
        std::cout << "\n🔌 Отключено от сервера" << std::endl;
    }

    std::cout << "👋 Клиент завершил работу" << std::endl;

    return 0;
}
